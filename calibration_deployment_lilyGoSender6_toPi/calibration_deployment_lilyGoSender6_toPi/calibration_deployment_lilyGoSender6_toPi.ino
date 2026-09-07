// sender_lilygo_wifi.ino
// LilyGO T3 V1.6.1 — one of these per camera. Talks to its ESP32-CAM over
// UART, and now hosts its OWN calibration webpage directly (this used to
// live on a separate "receiver" board that the sender pushed frames to
// over WiFi/TCP -- that hop is gone). Each board advertises itself under
// its own mDNS name (derived from its own WiFi MAC, so no two boards ever
// collide and nothing needs to be hand-configured per board), so
// calibrating N cameras at once is just opening N browser tabs, one per
// board's espcam-xxxx.local address.
//
// Once you're happy with a camera's lock/segment/recognize settings, hit
// Deploy on its page. That tears this board's WiFi down and switches it to
// LoRa-only: every capture cycle it radios out a small packet containing
// ITS OWN MAC ADDRESS plus the cam's status byte and recognized text, to
// whatever LoRa gateway is listening (receiver_lora_gateway.ino, one
// shared gateway for every deployed camera). Deployment is one-way and
// sticky by design -- getting back to calibration means power-cycling the
// board, which boots back into WiFi/calibration by default.
//
// ============================================================================
//  WHAT CHANGED FROM THE OLD 3-BOARD DESIGN
// ============================================================================
//  - No more separate "receiver" board for calibration. This board reads
//    frames from the cam over UART (same wire format as before) and serves
//    them straight from local RAM via its own WebServer -- no WiFi/TCP hop,
//    no frameServer, no pendingCommand-replied-on-next-push mechanism.
//  - Commands from the browser (/cmd?action=...) are relayed to the cam
//    over UART IMMEDIATELY, synchronously with the HTTP request. No more
//    waiting for the next frame push to carry a queued command down.
//  - LoRa packets are now tagged with this board's own MAC address (6
//    raw bytes) so ONE shared gateway can tell multiple deployed cameras
//    apart:
//        type(1)=0x03 | mac(6) | status(1) | textLen(1) | text(textLen)
//  - mDNS name is auto-derived from the last 2 bytes of this board's own
//    MAC ("espcam-<4 hex chars>"), so every board gets a unique, stable
//    address on the network with zero per-board configuration. The OLED
//    shows both the mDNS name and the raw IP at boot.
//  - The webpage itself (buttons, /frame.hex, /status, /cmd) is otherwise
//    unchanged from the old receiver's page -- it just now talks to this
//    board instead of a separate one.

#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <LoRa.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_mac.h>   // esp_read_mac() -- reads straight from efuse, works before WiFi is up

// ---------------------------------------------------------------------------
//  LoRa pins + radio settings (LilyGO T3 V1.6.1 / LoRa32 V2.1.6) -- must
//  match receiver_lora_gateway.ino exactly.
// ---------------------------------------------------------------------------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   23
#define LORA_DIO0  26

#define LORA_FREQ   868E6   // matches your module's silkscreen: 868/915MHz
#define LORA_SF     7
#define LORA_BW     125E3
#define LORA_CR     5

#define PKT_TYPE_TEXT  0x03   // deployment-mode packet: mac + status + recognized text

static bool loraOK = false;   // set true once LoRa.begin() succeeds in setup()

// ---------------------------------------------------------------------------
//  WiFi
// ---------------------------------------------------------------------------
const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "WIFI_Password";

// ---------------------------------------------------------------------------
//  OLED pins (I2C). No RST pin on this board. Failure here is non-fatal --
//  calibration still works over the network even without a screen, you'll
//  just need to check Serial (or your router's client list / an mDNS
//  browser tool) for this board's address instead.
// ---------------------------------------------------------------------------
#define OLED_SDA  21
#define OLED_SCL  22
Adafruit_SSD1306 display(128, 64, &Wire, -1);
static bool oledOK = false;

// ---------------------------------------------------------------------------
//  This board's own MAC -- captured once at boot, used both for the mDNS
//  name and stamped into every LoRa packet once deployed. Captured before
//  WiFi is ever torn down so it's still valid after Deploy switches WiFi off.
// ---------------------------------------------------------------------------
static uint8_t selfMac[6];
static char    mdnsName[24];   // "espcam-xxxx"
static char    macStr[18];     // "AA:BB:CC:DD:EE:FF" (for Serial logging only)

static void formatMac(const uint8_t *mac, char *out /* >=18 bytes */) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ---------------------------------------------------------------------------
//  UART to ESP32-CAM (same wiring as before)
// ---------------------------------------------------------------------------
#define CAM_RX_PIN 34   // sender RX  <- cam TX  (frames)
#define CAM_TX_PIN 12   // sender TX  -> cam RX  (commands)

#define CMD_SYNC              0xC0
#define CMD_DEPLOY            7      // handled locally, never relayed to the cam
#define CMD_SET_PANEL_THRESH  9      // relayed to the cam with a value byte
// Command byte values 1-6, 8, 9 match the cam's CMD_* constants 1:1:
// 1=LOCK, 2=UNLOCK, 3=SEGMENT_ON, 4=SEGMENT_OFF, 5=RECOGNIZE_ON,
// 6=RECOGNIZE_OFF, 7=DEPLOY (this board only, not sent to cam),
// 8=STREAM, 9=SET_PANEL_THRESH, 10=SHOW_MASK_ON, 11=SHOW_MASK_OFF.

static bool deploymentMode = false;   // false = calibration (WiFi + local webpage), true = LoRa-only

// ---------------------------------------------------------------------------
//  Locally-held frame state -- what the webpage reads from. Populated by
//  receiveFrame() below, straight off the UART link to the cam.
// ---------------------------------------------------------------------------
#define BUF_SIZE      9600
#define TEXT_MAX_LEN  48   // must match MAX_RECOGNIZED_CHARS on the cam and
                            // TEXT_MAX_LEN on the LoRa gateway

static uint8_t  frameBuf[BUF_SIZE];
static uint16_t frameW = 0, frameH = 0;
static uint8_t  frameStatus = 0;        // bit0=locked,1=segmented,2=recognized,3=streaming
static uint8_t  framePanelThresh = 0;
static char     frameText[TEXT_MAX_LEN + 1] = {0};
static uint8_t  frameTextLen = 0;
static volatile bool frameReady = false;      // sticky: true once ANY frame has ever arrived
static volatile bool newFrameForLora = false; // edge-triggered: true for exactly one loop() pass per frame
static portMUX_TYPE frameMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------------------------------------------------------------------
//  UART receiver state machine -- reads the cam's
//    0xAA 0x55 | status(1) | panelThresh(1) | w(2) | h(2) | packed data | textLen(1) | text
//  wire format directly off Serial2 and stages it into frameBuf/frameText.
// ---------------------------------------------------------------------------
enum RxState {
  RX_SYNC1, RX_SYNC2, RX_STATUS, RX_PANEL_THRESH, RX_W_HI, RX_W_LO, RX_H_HI, RX_H_LO,
  RX_DATA, RX_TEXT_LEN, RX_TEXT_DATA
};
static RxState  rxState   = RX_SYNC1;
static uint8_t  rxStatus;
static uint8_t  rxPanelThresh;
static uint16_t rxW, rxH;
static uint32_t rxExpected, rxReceived;
static uint8_t  rxStaging[BUF_SIZE];
static uint8_t  rxTextLen;
static uint8_t  rxTextReceived;
static uint8_t  rxTextStaging[TEXT_MAX_LEN];

static void commitFrame(uint8_t textLen) {
  static uint8_t lastLoggedStatus = 0xFF;   // impossible initial value forces one log at boot
  if (rxStatus != lastLoggedStatus) {
    Serial.printf("Cam status changed: 0x%02X -> 0x%02X (locked=%d segmented=%d recognized=%d streaming=%d)\n",
                  lastLoggedStatus, rxStatus,
                  (rxStatus & 0x01) != 0, (rxStatus & 0x02) != 0,
                  (rxStatus & 0x04) != 0, (rxStatus & 0x08) != 0);
    lastLoggedStatus = rxStatus;
  }
  portENTER_CRITICAL(&frameMux);
  memcpy(frameBuf, rxStaging, rxExpected);
  frameW = rxW; frameH = rxH; frameStatus = rxStatus;
  framePanelThresh = rxPanelThresh;
  frameTextLen = textLen;
  if (textLen > 0) memcpy(frameText, rxTextStaging, textLen);
  frameText[textLen] = '\0';
  frameReady = true;
  newFrameForLora = true;
  portEXIT_CRITICAL(&frameMux);
}

void receiveFrame() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    switch (rxState) {
      case RX_SYNC1:  rxState = (b == 0xAA) ? RX_SYNC2  : RX_SYNC1; break;
      case RX_SYNC2:  rxState = (b == 0x55) ? RX_STATUS : RX_SYNC1; break;
      case RX_STATUS: rxStatus = b;        rxState = RX_PANEL_THRESH; break;
      case RX_PANEL_THRESH:  rxPanelThresh = b;    rxState = RX_W_HI;  break;
      case RX_W_HI:   rxW  = (uint16_t)b << 8; rxState = RX_W_LO;   break;
      case RX_W_LO:   rxW |= b;                rxState = RX_H_HI;   break;
      case RX_H_HI:   rxH  = (uint16_t)b << 8; rxState = RX_H_LO;   break;
      case RX_H_LO:
        rxH |= b;
        rxExpected = ((rxW + 7) / 8) * (uint32_t)rxH;
        rxReceived = 0;
        if (rxExpected == 0 || rxExpected > BUF_SIZE) { rxState = RX_SYNC1; break; }
        rxState = RX_DATA;
        break;
      case RX_DATA:
        rxStaging[rxReceived++] = b;
        if (rxReceived >= rxExpected) rxState = RX_TEXT_LEN;
        break;
      case RX_TEXT_LEN:
        rxTextLen = b;
        if (rxTextLen == 0) {
          commitFrame(0);
          rxState = RX_SYNC1;
        } else if (rxTextLen > TEXT_MAX_LEN) {
          rxState = RX_SYNC1;   // malformed -- resync rather than overrun
        } else {
          rxTextReceived = 0;
          rxState = RX_TEXT_DATA;
        }
        break;
      case RX_TEXT_DATA:
        rxTextStaging[rxTextReceived++] = b;
        if (rxTextReceived >= rxTextLen) {
          commitFrame(rxTextLen);
          rxState = RX_SYNC1;
        }
        break;
    }
  }
}

// ---------------------------------------------------------------------------
//  Relay a command byte straight to the cam over UART. DEPLOY (7) never
//  reaches here -- it's intercepted in handleCmd() and handled locally.
// ---------------------------------------------------------------------------
void relayCommandToCam(uint8_t cmd, uint8_t val = 0) {
  if (cmd == 0 || cmd == CMD_DEPLOY || cmd > 11) return;
  Serial2.write(CMD_SYNC);
  Serial2.write(cmd);
  if (cmd == CMD_SET_PANEL_THRESH) Serial2.write(val);
}

// ---------------------------------------------------------------------------
//  LoRa init -- always initialized at boot (even though it's only used
//  once deployed) so it's ready the instant Deploy is pressed.
// ---------------------------------------------------------------------------
void initLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa FAILED -- deployment mode will be unavailable");
    loraOK = false;
    return;
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(0xF3);   // must match the gateway
  // Must match the gateway's LoRa.enableCrc() -- this is what makes every
  // outgoing packet carry a valid CRC for the gateway to check on receive,
  // so corrupted-in-transit packets get silently dropped by the radio
  // instead of reaching the gateway's onLoRaReceive() as garbage "devices".
  LoRa.enableCrc();
  loraOK = true;
  Serial.println("LoRa OK");
}

// ---------------------------------------------------------------------------
//  Tear down WiFi/webpage, switch to LoRa-only. One-way by design.
// ---------------------------------------------------------------------------
void enterDeploymentMode() {
  if (!loraOK) {
    Serial.println("DEPLOY requested but LoRa isn't available -- staying in calibration mode");
    return;
  }
  Serial.println("Entering deployment mode: WiFi off, LoRa-only from here on.");
  deploymentMode = true;
  if (oledOK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("DEPLOYED (LoRa-only)");
    display.println(macStr);
    display.println("Power-cycle to");
    display.println("re-enter calibration");
    display.display();
  }
  MDNS.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ---------------------------------------------------------------------------
//  Radio out this cycle's status + recognized text, tagged with this
//  board's own MAC so a shared gateway can tell cameras apart.
// ---------------------------------------------------------------------------
void sendTextLoRa() {
  if (!loraOK) return;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_TEXT);
  LoRa.write(selfMac, 6);
  LoRa.write(frameStatus);
  LoRa.write(frameTextLen);
  if (frameTextLen > 0) LoRa.write((const uint8_t*)frameText, frameTextLen);
  LoRa.endPacket();
}

// ---------------------------------------------------------------------------
//  HTTP handlers
// ---------------------------------------------------------------------------
WebServer server(80);

void handleCmd() {
  if (!server.hasArg("action")) { server.send(400, "text/plain", "missing action"); return; }
  String a = server.arg("action");
  uint8_t cmd = 0, val = 0;
  if      (a == "lock")          cmd = 1;
  else if (a == "unlock")        cmd = 2;
  else if (a == "segment_on")    cmd = 3;
  else if (a == "segment_off")   cmd = 4;
  else if (a == "recognize_on")  cmd = 5;
  else if (a == "recognize_off") cmd = 6;
  else if (a == "deploy")        cmd = 7;
  else if (a == "stream")        cmd = 8;
  else if (a == "set_panel_thresh") {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "missing value"); return; }
    int v = server.arg("value").toInt();
    if (v < 0 || v > 255) { server.send(400, "text/plain", "value out of range"); return; }
    val = (uint8_t)v;
    cmd = 9;
  }
  else { server.send(400, "text/plain", "unknown action"); return; }

  Serial.printf("HTTP /cmd action=%s -> cmd=%u val=%u\n", a.c_str(), cmd, val);

  if (cmd == CMD_DEPLOY) {
    enterDeploymentMode();
  } else {
    relayCommandToCam(cmd, val);
    Serial.printf("Relayed to cam over Serial2: 0xC0 0x%02X%s\n", cmd,
                  (cmd == CMD_SET_PANEL_THRESH) ? " + value byte" : "");
  }
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  uint8_t st, panelThresh;
  bool ready;
  char textCopy[TEXT_MAX_LEN + 1];

  portENTER_CRITICAL(&frameMux);
  st = frameStatus;
  panelThresh = framePanelThresh;
  ready = frameReady;
  strncpy(textCopy, frameText, TEXT_MAX_LEN);
  textCopy[TEXT_MAX_LEN] = '\0';
  portEXIT_CRITICAL(&frameMux);

  bool locked     = st & 0x01;
  bool segmented  = st & 0x02;
  bool recognized = st & 0x04;
  bool streaming  = st & 0x08;

  String textJson;
  textJson.reserve(strlen(textCopy) + 8);
  for (const char *p = textCopy; *p; p++) {
    if (*p == '\n') textJson += "\\n";
    else textJson += *p;
  }

  String json = String("{\"frameReady\":") + (ready ? "true" : "false") +
                ",\"locked\":" + (locked ? "true" : "false") +
                ",\"segmented\":" + (segmented ? "true" : "false") +
                ",\"recognized\":" + (recognized ? "true" : "false") +
                ",\"streaming\":" + (streaming ? "true" : "false") +
                ",\"loraReady\":" + (loraOK ? "true" : "false") +
                ",\"panelThresh\":" + String((int)panelThresh) +
                ",\"text\":\"" + textJson + "\"" +
                ",\"mac\":\"" + String(macStr) + "\"}";
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", json);
}

const char VIEWER_HTML[] PROGMEM = R"html(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ESP32-CAM Calibration</title>
  <style>
    body{background:#111;display:flex;flex-direction:column;align-items:center;
         justify-content:center;min-height:100vh;margin:0;font-family:monospace;color:#eee;}
    h2{margin-bottom:2px;letter-spacing:2px;}
    #macLabel{font-size:11px;color:#666;margin-bottom:10px;}
    canvas{image-rendering:pixelated;border:2px solid #444;}
    #status{margin-top:10px;font-size:12px;color:#888;}
    #mode{margin-top:6px;font-size:13px;color:#6cf;}
    #recognizedText{margin-top:12px;font-size:28px;letter-spacing:3px;color:#9f6;
                    min-height:34px;white-space:pre-line;line-height:1.3;}
    #loraNote{margin-top:4px;font-size:12px;color:#f93;min-height:16px;}
    #controls{margin-top:16px;display:flex;gap:8px;flex-wrap:wrap;justify-content:center;max-width:360px;}
    button{background:#222;color:#eee;border:1px solid #555;border-radius:4px;
           padding:8px 14px;font-family:monospace;font-size:13px;cursor:pointer;}
    button:hover:not(:disabled){background:#333;}
    button:disabled{opacity:0.35;cursor:default;}
    button.active{border-color:#6cf;color:#6cf;}
    #btnDeploy{border-color:#f93;color:#f93;}
  </style>
</head>
<body>
  <h2>ESP32-CAM Calibration</h2>
  <div id="macLabel"></div>
  <canvas id="bw-canvas"></canvas>
  <div id="recognizedText">Reading: —</div>
  <div id="loraNote"></div>
  <div id="status">Waiting for first frame...</div>
  <div id="mode">Mode: —</div>
  <div id="threshRow" style="margin-top:14px;display:flex;align-items:center;gap:8px;">
    <label for="panelThresh" style="font-size:13px;color:#aaa;">Panel Thresh</label>
    <input type="range" id="panelThresh" min="0" max="255" value="185">
    <span id="panelThreshVal" style="font-size:13px;color:#6cf;width:32px;">185</span>
  </div>
  <div id="controls">
    <button id="btnStream">Stream (Raw)</button>
    <button id="btnLock">Lock Coordinates</button>
    <button id="btnUnlock">Unlock / Resume Search</button>
    <button id="btnSegOn">Segment On</button>
    <button id="btnSegOff">Segment Off</button>
    <button id="btnRecOn">Recognize On</button>
    <button id="btnRecOff">Recognize Off</button>
    <button id="btnDeploy">Deploy (LoRa)</button>
  </div>
  <script>
    document.title = location.hostname + ' - ESP32-CAM';
    const canvas = document.getElementById('bw-canvas');
    const ctx    = canvas.getContext('2d');
    const status = document.getElementById('status');
    const modeEl = document.getElementById('mode');
    const macEl  = document.getElementById('macLabel');
    const recognizedEl = document.getElementById('recognizedText');
    const loraNoteEl = document.getElementById('loraNote');
    const btnLock   = document.getElementById('btnLock');
    const btnUnlock = document.getElementById('btnUnlock');
    const btnSegOn  = document.getElementById('btnSegOn');
    const btnSegOff = document.getElementById('btnSegOff');
    const btnRecOn  = document.getElementById('btnRecOn');
    const btnRecOff = document.getElementById('btnRecOff');
    const btnDeploy = document.getElementById('btnDeploy');
    const btnStream = document.getElementById('btnStream');
    const threshSlider = document.getElementById('panelThresh');
    const threshVal = document.getElementById('panelThreshVal');
    let threshDragging = false;

    function sendCmd(action, value) {
      let url = '/cmd?action=' + action;
      if (value !== undefined) url += '&value=' + value;
      fetch(url).catch(() => {});
    }
    threshSlider.oninput = () => { threshVal.textContent = threshSlider.value; };
    threshSlider.onmousedown = threshSlider.ontouchstart = () => { threshDragging = true; };
    threshSlider.onchange = () => { sendCmd('set_panel_thresh', threshSlider.value); threshDragging = false; };
    btnStream.onclick = () => sendCmd('stream');
    btnLock.onclick   = () => sendCmd('lock');
    btnUnlock.onclick = () => sendCmd('unlock');
    btnSegOn.onclick  = () => sendCmd('segment_on');
    btnSegOff.onclick = () => sendCmd('segment_off');
    btnRecOn.onclick  = () => sendCmd('recognize_on');
    btnRecOff.onclick = () => sendCmd('recognize_off');
    btnDeploy.onclick = () => {
      if (confirm('Deploy switches this board to LoRa-only and turns its WiFi off. ' +
                   'This page will stop responding immediately, and the only way back ' +
                   'to calibration is to power-cycle the board. Continue?')) {
        sendCmd('deploy');
      }
    };

    function refreshFrame() {
      fetch('/frame.hex?t=' + Date.now())
        .then(r => { if (!r.ok) throw new Error('no frame'); return r.text(); })
        .then(hex => {
          const parts = hex.split(',');
          const W = parseInt(parts[0]), H = parseInt(parts[1]);
          const data = parts[2];
          const ROW_BYTES = Math.ceil(W / 8);
          canvas.width  = W;
          canvas.height = H;
          canvas.style.width  = Math.min(W * 8, 384) + 'px';
          canvas.style.height = Math.min(H * 8, 504) + 'px';
          const img = ctx.createImageData(W, H);
          for (let y = 0; y < H; y++) {
            for (let x = 0; x < W; x++) {
              const byteIdx = y * ROW_BYTES + Math.floor(x / 8);
              const bitIdx  = 7 - (x % 8);
              const byteVal = parseInt(data.substr(byteIdx * 2, 2), 16);
              const val     = ((byteVal >> bitIdx) & 1) ? 255 : 0;
              const px      = (y * W + x) * 4;
              img.data[px] = img.data[px+1] = img.data[px+2] = val;
              img.data[px+3] = 255;
            }
          }
          ctx.putImageData(img, 0, 0);
          status.textContent = 'Updated: ' + new Date().toLocaleTimeString();
        })
        .catch(() => { status.textContent = 'Waiting for frame...'; });
    }

    function refreshStatus() {
      fetch('/status?t=' + Date.now())
        .then(r => r.json())
        .then(s => {
          macEl.textContent = s.mac || '';
          if (!threshDragging && typeof s.panelThresh === 'number') {
            threshSlider.value = s.panelThresh;
            threshVal.textContent = s.panelThresh;
          }
          const extra = [];
          if (s.segmented)  extra.push('SEGMENTED');
          if (s.recognized) extra.push('RECOGNIZED');
          const modeLabel = s.streaming ? 'STREAMING (raw preview)' : (s.locked ? 'LOCKED' : 'SEARCHING');
          modeEl.textContent = 'Mode: ' + modeLabel +
                                (s.locked ? (extra.length ? ' + ' + extra.join(' + ') : ' (plain)') : '');

          const searching = !s.locked && !s.streaming;
          btnStream.disabled = s.streaming;
          btnLock.disabled   = s.locked || s.streaming;
          btnUnlock.disabled = searching;
          btnSegOn.disabled  = !s.locked || s.segmented;
          btnSegOff.disabled = !s.locked || !s.segmented;
          btnRecOn.disabled  = !s.locked || s.recognized;
          btnRecOff.disabled = !s.locked || !s.recognized;
          btnDeploy.disabled = !s.locked || !s.recognized;

          btnStream.classList.toggle('active', s.streaming);
          btnLock.classList.toggle('active', s.locked);
          btnSegOn.classList.toggle('active', s.segmented);
          btnRecOn.classList.toggle('active', s.recognized);

          recognizedEl.textContent = (s.recognized && s.text) ? ('Reading:\n' + s.text) : 'Reading: —';
          loraNoteEl.textContent = s.loraReady ? '' :
            "This board's LoRa radio isn't responding -- Deploy will be unavailable until that's fixed.";
        })
        .catch(() => { modeEl.textContent = 'Mode: unknown'; });
    }

    refreshFrame();
    refreshStatus();
    setInterval(refreshFrame, 6000);
    setInterval(refreshStatus, 500);
  </script>
</body>
</html>
)html";

void handleRoot() {
  server.send_P(200, "text/html", VIEWER_HTML);
}

void handleFrameHex() {
  if (!frameReady) { server.send(503, "text/plain", ""); return; }

  static uint8_t localBuf[BUF_SIZE];
  uint16_t w, h;
  uint32_t ps;

  portENTER_CRITICAL(&frameMux);
  w  = frameW;
  h  = frameH;
  ps = ((w + 7) / 8) * (uint32_t)h;
  memcpy(localBuf, frameBuf, ps);
  portEXIT_CRITICAL(&frameMux);

  String resp = String(w) + "," + String(h) + ",";
  resp.reserve(resp.length() + ps * 2);
  char tmp[3];
  for (uint32_t i = 0; i < ps; i++) {
    sprintf(tmp, "%02x", localBuf[i]);
    resp += tmp;
  }
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/plain", resp);
}

// ---------------------------------------------------------------------------
void showBootScreen(const String &ip) {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESP32-CAM Calibration");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println("Open browser at:");
  display.setCursor(0, 28);
  display.println(String(mdnsName) + ".local");
  display.setCursor(0, 40);
  display.println("or " + ip);
  display.setCursor(0, 54);
  display.println(macStr);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  delay(200);

  Wire.begin(OLED_SDA, OLED_SCL);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOK) Serial.println("OLED not found -- continuing without it");

  Serial.println("Starting LoRa...");
  initLoRa();

  // Read the MAC straight from efuse -- WiFi.macAddress() can return all
  // zeros if called before the WiFi driver has fully initialized, which
  // silently breaks both mDNS naming (collisions between boards) and LoRa
  // device identification (every board looks like the same device to the
  // gateway). esp_read_mac() has no such ordering dependency.
  esp_read_mac(selfMac, ESP_MAC_WIFI_STA);
  formatMac(selfMac, macStr);
  WiFi.mode(WIFI_STA);
  snprintf(mdnsName, sizeof(mdnsName), "espcam-%02x%02x", selfMac[4], selfMac[5]);
  Serial.printf("This board's MAC: %s   mDNS name: %s.local\n", macStr, mdnsName);

  Serial.println("Connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts > 40) { Serial.println("\nWiFi FAILED - will keep retrying in loop()"); break; }
  }

  String ip = "not connected";
  if (WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP().toString();
    Serial.println("\nWiFi OK, IP: " + ip);
    if (MDNS.begin(mdnsName)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("mDNS OK: reachable as %s.local\n", mdnsName);
    } else {
      Serial.println("mDNS FAILED -- use the raw IP instead");
    }
  }
  showBootScreen(ip);

  server.on("/",          handleRoot);
  server.on("/frame.hex", handleFrameHex);
  server.on("/cmd",       handleCmd);
  server.on("/status",    handleStatus);
  server.begin();
  Serial.println("Calibration webpage ready.");
}

void loop() {
  if (!deploymentMode) {
    static uint32_t lastReconnectAttempt = 0;
    if (WiFi.status() != WL_CONNECTED && millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    server.handleClient();
  }

  receiveFrame();

  if (newFrameForLora) {
    newFrameForLora = false;
    if (deploymentMode) {
      sendTextLoRa();
      Serial.printf("Deployment: sent status=0x%02X text=\"%s\"\n", frameStatus, frameText);
    }
  }
}
