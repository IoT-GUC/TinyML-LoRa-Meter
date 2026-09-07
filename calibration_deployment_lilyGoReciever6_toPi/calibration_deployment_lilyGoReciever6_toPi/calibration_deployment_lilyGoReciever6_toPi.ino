// receiver_lora_gateway.ino
// LilyGO T3 V1.6.1 — "Board 3". Pure LoRa-to-serial gateway: listens for
// PKT_TYPE_TEXT packets from any number of deployed sender boards (each
// sender_lilygo_wifi.ino stamps its own MAC into every packet), assigns
// each new MAC a friendly name the first time it's seen (Device-1,
// Device-2, ...), and forwards ONE JSON LINE PER PACKET over USB serial to
// the Pi:
//     {"mac":"AA:BB:CC:DD:EE:FF","name":"Device-1","data":{...},"rssi":-63}
//
// This is the only thing printed on Serial -- pi_server_lora.py reads it
// line by line and does json.loads() on each one, so nothing else should
// share this UART. (Debug logging, if you want it, should go to Serial1/
// Serial2 on a spare UART instead of polluting this one.)
//
// No WiFi, no webpage, no per-board configuration needed -- this board
// doesn't care how many cameras are deployed or which ones; it just
// listens and forwards. Add cameras by flashing more sender boards; this
// board doesn't need to change.
//
// ============================================================================
//  WHY THIS REPLACES THE OLD receiver_lilygo_wifi.ino
// ============================================================================
// The old receiver did double duty: it hosted the calibration webpage AND
// relayed LoRa readings once deployed. Now that every sender hosts its own
// calibration webpage directly, this board only needs the second half of
// that job -- and it needs to handle MANY senders at once instead of one,
// which is why readings are now tagged by MAC and named on first sight
// instead of assuming a single hardcoded sender.

#include <SPI.h>
#include <LoRa.h>

// ---------------------------------------------------------------------------
//  LoRa pins + radio settings (LilyGO T3 V1.6.1 / LoRa32 V2.1.6) -- must
//  match sender_lilygo_wifi.ino exactly.
// ---------------------------------------------------------------------------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   23
#define LORA_DIO0  26

#define LORA_FREQ   868E6
#define LORA_SF     7
#define LORA_BW     125E3
#define LORA_CR     5

#define PKT_TYPE_TEXT  0x03   // type(1) | mac(6) | status(1) | textLen(1) | text(textLen)

#define TEXT_MAX_LEN  48      // must match MAX_RECOGNIZED_CHARS on the cam and
                               // TEXT_MAX_LEN on every sender

// ---------------------------------------------------------------------------
//  Per-MAC friendly-name registry. Linear scan is plenty for a handful of
//  cameras; bump MAX_DEVICES if you deploy more than that. Names are
//  assigned in first-seen order and kept only in RAM -- they'll renumber
//  if this board reboots. If you want names stable across reboots, persist
//  this table with Preferences.h instead.
// ---------------------------------------------------------------------------
#define MAX_DEVICES 24

struct DeviceEntry {
  uint8_t mac[6];
  char    name[16];
};
static DeviceEntry devices[MAX_DEVICES];
static int deviceCount = 0;

static bool macEquals(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

static void formatMac(const uint8_t *mac, char *out /* >=18 bytes */) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Returns the friendly name for this MAC, assigning a new one the first
// time it's seen. Falls back to the raw MAC as the "name" if the table is
// full, rather than silently dropping readings from a real device.
static const char* nameForMac(const uint8_t *mac) {
  for (int i = 0; i < deviceCount; i++) {
    if (macEquals(devices[i].mac, mac)) return devices[i].name;
  }
  if (deviceCount < MAX_DEVICES) {
    DeviceEntry &e = devices[deviceCount];
    memcpy(e.mac, mac, 6);
    snprintf(e.name, sizeof(e.name), "Device-%d", deviceCount + 1);
    deviceCount++;
    return e.name;
  }
  static char fallback[18];
  formatMac(mac, fallback);
  return fallback;
}

// ---------------------------------------------------------------------------
//  Received-packet staging. onLoRaReceive() runs in interrupt context (the
//  LoRa library fires it off the DIO0 pin), so it does the minimum work
//  possible -- copy the raw bytes out and set a flag -- and everything
//  that touches Serial/String/Serial-of-JSON happens in loop() instead.
// ---------------------------------------------------------------------------
static volatile bool    pktReady = false;
static volatile uint8_t pktMac[6];
static volatile uint8_t pktStatus;
static volatile uint8_t pktTextLen;
static volatile char    pktText[TEXT_MAX_LEN + 1];
static volatile int     pktRssi;

void onLoRaReceive(int pktSize) {
  // header is type(1)+mac(6)+status(1)+textLen(1) = 9 bytes minimum
  if (pktSize < 9) { while (LoRa.available()) LoRa.read(); return; }
  if (pktReady) { while (LoRa.available()) LoRa.read(); return; }  // previous packet not drained yet -- drop this one

  uint8_t type = LoRa.read();
  if (type != PKT_TYPE_TEXT) { while (LoRa.available()) LoRa.read(); return; }

  uint8_t mac[6];
  for (int i = 0; i < 6; i++) mac[i] = LoRa.read();
  uint8_t status  = LoRa.read();
  uint8_t textLen = LoRa.read();
  if (textLen > TEXT_MAX_LEN) { while (LoRa.available()) LoRa.read(); return; }

  char text[TEXT_MAX_LEN + 1];
  uint8_t i = 0;
  while (LoRa.available() && i < textLen) text[i++] = LoRa.read();
  text[i] = '\0';
  while (LoRa.available()) LoRa.read();   // drain anything unexpected/extra

  memcpy((void*)pktMac, mac, 6);
  pktStatus  = status;
  pktTextLen = textLen;
  memcpy((void*)pktText, text, textLen + 1);
  pktRssi = LoRa.packetRssi();
  pktReady = true;
}

void initLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    while (true) delay(1000);   // nothing useful this board can do without LoRa
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(0xF3);   // must match every sender
  // Enables the radio chip's own hardware CRC check. Without this, any
  // packet that merely matches the 1-byte sync word gets delivered to
  // onLoRaReceive() even if the payload was corrupted in the air -- which
  // is how corrupted packets end up looking like brand-new "devices" with
  // garbage MACs. Must be enabled here AND on every sender (sendTextLoRa's
  // initLoRa()) -- CRC is computed on send, checked on receive.
  LoRa.enableCrc();
  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();
}

// ---------------------------------------------------------------------------
//  Build and print one JSON line for the Pi. Runs in loop(), not the ISR.
// ---------------------------------------------------------------------------
void emitJson() {
  uint8_t mac[6];
  memcpy(mac, (const void*)pktMac, 6);
  char macBuf[18];
  formatMac(mac, macBuf);
  const char *name = nameForMac(mac);

  char textBuf[TEXT_MAX_LEN + 1];
  memcpy(textBuf, (const void*)pktText, pktTextLen + 1);

  // textBuf only ever contains '0'-'9', '.', '?', or '\n' row separators
  // (see the cam's DigitClassifier / row-grouping step) -- only '\n' needs
  // JSON escaping.
  String textJson;
  textJson.reserve(strlen(textBuf) + 8);
  for (const char *p = textBuf; *p; p++) {
    if (*p == '\n') textJson += "\\n";
    else textJson += *p;
  }

  bool locked     = pktStatus & 0x01;
  bool segmented  = pktStatus & 0x02;
  bool recognized = pktStatus & 0x04;
  bool streaming  = pktStatus & 0x08;

  String json = String("{\"mac\":\"") + macBuf + "\"" +
                ",\"name\":\"" + name + "\"" +
                ",\"data\":{" +
                  "\"text\":\"" + textJson + "\"" +
                  ",\"locked\":" + (locked ? "true" : "false") +
                  ",\"segmented\":" + (segmented ? "true" : "false") +
                  ",\"recognized\":" + (recognized ? "true" : "false") +
                  ",\"streaming\":" + (streaming ? "true" : "false") +
                "}" +
                ",\"rssi\":" + String(pktRssi) +
                "}";
  Serial.println(json);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  initLoRa();
}

void loop() {
  if (pktReady) {
    emitJson();
    pktReady = false;
  }
}
