"""
Raspberry Pi central server - LoRa version, with dynamic ThingsBoard provisioning.

Board 1 and Board 2 (and any future boards) send sensor packets over LoRa
to Board 3. Board 3 is connected to this Pi over USB and forwards each
packet as one line of JSON on Serial: {"mac", "name", "data", "rssi"}.

No MAC addresses are hardcoded anywhere. The first time this script sees
a new MAC, it automatically provisions a brand-new device in the locally
running ThingsBoard instance via the provisioning API, gets back an
access token, and remembers it (both in memory and in device_tokens.json
on disk) so it never re-provisions the same MAC twice, even across
restarts. From then on, telemetry for that MAC is pushed straight to its
ThingsBoard device.

Everything talks to ThingsBoard over http://localhost:8080 (loopback),
so no WiFi/network is required for this pipeline.

Run with: python3 pi_server_lora.py
Requires: pip install flask pyserial requests

Find Board 3's serial port first, e.g.:
    ls /dev/serial/by-id/      (Linux, most reliable - stable name)
    ls /dev/ttyUSB* /dev/ttyACM*

ThingsBoard setup required before running (one-time):
    1. Profiles -> Device profiles -> open (or create) a profile.
    2. Provisioning tab -> strategy = "Allow to create new devices".
    3. Copy the generated Provision device key + secret into the
       PROVISION_DEVICE_KEY / PROVISION_DEVICE_SECRET constants below.
"""

import json
import os
import re
import shutil
import threading
import time
from datetime import datetime

import requests
import serial
from flask import Flask, jsonify

# ----- SERIAL CONFIG -----
SERIAL_PORT = "/dev/ttyUSB0"   # change to match Board 3's port
BAUD_RATE = 115200
# --------------------------

# ----- THINGSBOARD CONFIG -----
# localhost = loopback interface, never leaves the Pi, no WiFi/network needed
THINGSBOARD_URL = "http://localhost:8080"

# From the device profile's Provisioning tab in the ThingsBoard UI
PROVISION_DEVICE_KEY = "REPLACE_ME"
PROVISION_DEVICE_SECRET = "REPLACE_ME"

# Where provisioned MAC -> token mappings are cached across restarts
TOKENS_FILE = "device_tokens.json"

# Only auto-provision devices the gateway actually named "Device-N".
# If the gateway's per-MAC name table fills up (or a corrupted/garbled
# LoRa packet somehow slips past the radio's CRC check), it falls back to
# using the raw MAC as the "name" -- that's a signal something's wrong,
# not a real new camera, so we refuse to provision it. Reading still
# shows up in latest_readings / /status for visibility, it just never
# creates a ThingsBoard device or burns a token.
DEVICE_NAME_PATTERN = re.compile(r"^Device-\d+$")
# --------------------------------

app = Flask(__name__)

latest_readings = {}   # mac -> last entry received
mac_to_token = {}       # mac -> ThingsBoard access token (cache)
_lock = threading.Lock()
_tokens_lock = threading.Lock()


def load_tokens():
    """Loads the MAC->token cache from disk, if it exists."""
    global mac_to_token
    if os.path.exists(TOKENS_FILE):
        try:
            with open(TOKENS_FILE, "r") as f:
                mac_to_token = json.load(f)
            print(f"Loaded {len(mac_to_token)} cached device token(s) from {TOKENS_FILE}")
        except (json.JSONDecodeError, OSError) as e:
            print(f"Could not read {TOKENS_FILE} ({e}), starting with an empty cache")
            mac_to_token = {}


def save_tokens():
    """Persists the current MAC->token cache to disk."""
    with _tokens_lock:
        try:
            with open(TOKENS_FILE, "w") as f:
                json.dump(mac_to_token, f, indent=2)
        except OSError as e:
            print(f"Could not save {TOKENS_FILE}: {e}")


def provision_device(mac, name):
    """Asks ThingsBoard to create a brand-new device for this MAC and
    returns the access token it assigns, or None on failure."""
    url = f"{THINGSBOARD_URL}/api/v1/provision"
    body = {
        "deviceName": name or mac,
        "provisionDeviceKey": PROVISION_DEVICE_KEY,
        "provisionDeviceSecret": PROVISION_DEVICE_SECRET,
        "credentialsType": "ACCESS_TOKEN",
    }
    try:
        r = requests.post(url, json=body, timeout=5)
        r.raise_for_status()
        result = r.json()
        if result.get("status") != "SUCCESS":
            print(f"Provisioning rejected for {mac}: {result}")
            return None
        token = result.get("credentialsValue")
        print(f"Provisioned new ThingsBoard device for {mac} ({name}) -> token {token}")
        return token
    except requests.RequestException as e:
        print(f"Provisioning request failed for {mac}: {e}")
        return None


def get_token_for(mac, name):
    """Returns a cached token for this MAC, provisioning a new device
    on ThingsBoard the first time this MAC is ever seen -- but only if
    the gateway gave it a proper Device-N name. See DEVICE_NAME_PATTERN
    above for why."""
    with _tokens_lock:
        token = mac_to_token.get(mac)
    if token:
        return token

    if not name or not DEVICE_NAME_PATTERN.match(name):
        print(f"Refusing to provision {mac}: name '{name}' doesn't look like a "
              f"real gateway-assigned device (expected 'Device-N'). Likely a "
              f"corrupted packet or a full gateway name table -- not creating "
              f"a ThingsBoard device for it.")
        return None

    token = provision_device(mac, name)
    if token:
        with _tokens_lock:
            mac_to_token[mac] = token
        save_tokens()
    return token


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def print_table():
    """Redraws the terminal with a 4-row block per known device."""
    clear_screen()
    width = shutil.get_terminal_size((100, 20)).columns

    with _lock:
        entries = sorted(latest_readings.values(), key=lambda e: e["name"])

    print("=" * width)
    if not entries:
        print("Waiting for data from Board 3...")
        print("=" * width)
        return

    for entry in entries:
        ts = entry["received_at"].split("T")[1].split(".")[0]  # HH:MM:SS
        print(f"{entry['name']}  ({entry['mac']})  rssi={entry['rssi']}  last={ts}")
        print("-" * width)

        data = entry["data"]
        if not data:
            print("  (no data yet)")
        else:
            for key, value in data.items():
                print(f"  {key:<10}: {value}")

        print("=" * width)


def push_to_thingsboard(mac, name, data):
    """Gets (or provisions) a token for this MAC and posts telemetry.
    Silently skips if there's no data, the name failed validation, or
    provisioning failed."""
    if not data:
        return
    token = get_token_for(mac, name)
    if not token:
        return  # provisioning failed or was refused, already logged
    url = f"{THINGSBOARD_URL}/api/v1/{token}/telemetry"
    try:
        requests.post(url, json=data, timeout=2)
    except requests.RequestException as e:
        print(f"ThingsBoard push failed for {mac}: {e}")


def serial_reader():
    """Runs forever in a background thread, reading one JSON line at
    a time from Board 3 and updating latest_readings, keyed by MAC."""
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                print(f"Connected to Board 3 on {SERIAL_PORT}")
                while True:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    try:
                        payload = json.loads(line)
                    except json.JSONDecodeError:
                        continue  # skip corrupted/partial lines

                    if not isinstance(payload, dict) or "mac" not in payload:
                        continue  # skip anything that isn't the {mac, name, data, ...} object we expect

                    mac = payload["mac"]
                    name = payload.get("name", "Unregistered")
                    entry = {
                        "name": name,
                        "mac": mac,
                        "data": payload.get("data", {}),
                        "rssi": payload.get("rssi"),
                        "received_at": datetime.now().isoformat(),
                    }
                    with _lock:
                        latest_readings[mac] = entry

                    push_to_thingsboard(mac, name, entry["data"])
                    print_table()

        except serial.SerialException as e:
            print(f"Serial error ({e}), retrying in 3s...")
            time.sleep(3)


@app.route("/status", methods=["GET"])
def status():
    with _lock:
        return jsonify(latest_readings), 200


if __name__ == "__main__":
    load_tokens()

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # 0.0.0.0 so /status is reachable from other machines on the LAN
    app.run(host="0.0.0.0", port=5000, debug=False)
