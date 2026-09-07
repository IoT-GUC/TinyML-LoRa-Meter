/*
 * Auto-detecting numbers-region capture + BW send for ESP32-CAM, with a
 * lockable crop, optional per-digit segmentation, and optional on-device
 * digit recognition -- all controlled remotely from the webpage.
 *
 * ============================================================================
 *  NEW IN THIS VERSION (on top of the auto-detect + lock/segment code)
 * ============================================================================
 *  - A third locked-mode feature: RECOGNIZE. When on, the cam classifies
 *    every kept digit blob with DigitClassifier (digit_classifier_2.h),
 *    sorts them left-to-right (ascending x0 in crop-local coords -- i.e.
 *    reading order for a single-row numeric display), and sends the
 *    resulting string alongside the frame. recognizeOn is independent of
 *    segmentOn: either, both, or neither can be active while locked.
 *  - Blob detection itself is now shared between the two features: it
 *    runs whenever segmentOn OR recognizeOn is set, so turning on
 *    recognition alone doesn't skip finding the boxes it needs, and
 *    turning on segmentation alone doesn't run the classifier.
 *  - New command bytes (still 0xC0 <cmd> on the same UART link):
 *        5 = RECOGNIZE_ON, 6 = RECOGNIZE_OFF
 *    LOCK and UNLOCK both reset recognizeOn to false, same as segmentOn,
 *    so every fresh lock starts plain.
 *  - New STATUS bit:
 *        bit2 (0x04) = recognized (only set on frames where the text
 *                      actually came out non-empty -- same "only true
 *                      when it actually happened this frame" convention
 *                      as the segmented bit)
 *  - New wire format:
 *        0xAA 0x55 | status(1) | w(2) | h(2) | packed data | textLen(1) | text(textLen)
 *    textLen is always sent (0 when recognizeOn is off or found nothing),
 *    so downstream parsers don't need to branch on the status bits to
 *    know how many bytes are coming.
 *
 *  HARDWARE: this requires the sender board's TX pin (GPIO12 in
 *  sender_lilygo_wifi.ino) to be wired to this board's UART RX pin
 *  (GPIO3 / U0R on AI-Thinker ESP32-CAM). That pin doubles as the USB
 *  flashing RX line -- disconnect the wire before reflashing the cam.
 *
 * Requires PSRAM (~1MB free) for the working buffers below, plus the
 * classifier's own ~20KB tensor arena (internal RAM, not PSRAM).
 *
 *  NEW: a fourth mode, MODE_STREAM, is now the board's default state at
 *  boot (before any command arrives). It skips ALL detection -- no panel
 *  search, no lock, no segment, no recognize -- and just BW-thresholds
 *  and packs the ENTIRE raw QVGA frame every cycle, so the webpage can
 *  show "what the cam is actually looking at" before you trust the
 *  auto-detect pipeline to find anything in it. It reuses the exact same
 *  wire format as every other mode (a full frame packs to exactly 9600
 *  bytes -- the same size every buffer here was already sized for), so
 *  no protocol changes were needed for it downstream. New status bit3
 *  (0x08) = streaming. Command 8 = STREAM, switches into it from any
 *  mode; the existing UNLOCK command (2) already unconditionally sets
 *  MODE_SEARCHING regardless of prior mode, so it doubles as "start
 *  searching" from MODE_STREAM with no changes needed there.
 */

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include <string.h>
#include "board_config.h"       // AI-Thinker pin map (Y2..Y9, XCLK, etc.)
#include "digit_classifier_2.h" // NEW: on-device digit recognition
#include <math.h>   // fabsf(), used by the decimal-point baseline check

// ---------------------------------------------------------------------------
//  DETECTION PARAMETERS  — mirror numbers_region_detector.py exactly
// ---------------------------------------------------------------------------
#define PANEL_THRESH       185   // brightness cutoff to find the panel
#define MORPH_RADIUS       7     // close/open kernel radius (px)
#define BLOCK_SIZE         35    // local-mean window for ink detection (must be odd)
#define LOCAL_OFFSET       10    // local-mean offset for ink detection
#define MAX_BAR_FRACTION   0.6f  // discard blobs wider/taller than this fraction
                                  // of the reference region (header bars/borders)
#define REGION_PADDING      4     // extra px padding around the detected region
#define MAX_COMPONENTS     300   // cap on ink blobs tracked per frame
#define ROW_GAP_FRACTION   0.6f  // a gap between consecutive (y-sorted) glyph
                                  // vertical centers bigger than this fraction
                                  // of the average glyph height starts a new row
#define DOT_ROW_MARGIN_FRACTION  0.5f   // vertical slack (fraction of the row's avg digit
                                          // height) used to decide if a small candidate blob
                                          // belongs to this row at all
#define DOT_MAX_SIZE_FRACTION    0.35f  // a candidate wider/taller than this fraction of the
                                          // row's avg digit height is too big to be a dot
#define DOT_BASELINE_TOLERANCE   0.30f  // and its bottom edge (y1) must sit within this
                                          // fraction of avg digit height of the row's baseline
// ---------------------------------------------------------------------------
//  BLACK & WHITE PARAMETERS  — must match quality_analyzer.py exactly
//    threshold_local(gray_np, block_size=35, offset=10)
// ---------------------------------------------------------------------------
#define BW_BLOCK_SIZE      35    // must be odd
#define BW_OFFSET          10

// ---------------------------------------------------------------------------
//  RECOGNITION PARAMETERS
// ---------------------------------------------------------------------------
#define MAX_RECOGNIZED_CHARS 48  // ceiling on how many classified glyph/row-
                                  // separator chars we send per frame; must
                                  // match TEXT_MAX_LEN in
                                  // sender_lilygo_wifi.ino and
                                  // receiver_lilygo_wifi.ino
#define ROW_GAP_FRACTION   0.6f  // a gap between consecutive (y-sorted) glyph
                                  // vertical centers bigger than this fraction
                                  // of the average glyph height starts a new row

// ---------------------------------------------------------------------------
//  Capture frame size — fixed, so all working buffers can be sized once
// ---------------------------------------------------------------------------
#define FRAME_SIZE   FRAMESIZE_QVGA
#define FRAME_W      320
#define FRAME_H      240

// ---------------------------------------------------------------------------
//  Working buffers (allocated once in setup(), all in PSRAM, all reused
//  every capture — nothing is malloc'd inside the capture loop)
// ---------------------------------------------------------------------------
static uint8_t  *maskBuf     = nullptr;  // FRAME_W*FRAME_H, panel mask / morphology ping-pong
static uint8_t  *maskBuf2    = nullptr;  // FRAME_W*FRAME_H
static uint32_t *integralBuf = nullptr;  // (FRAME_W+1)*(FRAME_H+1), reused for every integral image
static uint8_t  *visitedBuf  = nullptr;  // FRAME_W*FRAME_H, connected-component visited flags
static uint32_t *stackBuf    = nullptr;  // FRAME_W*FRAME_H, flood-fill stack
static uint8_t  *roiBuf      = nullptr;  // FRAME_W*FRAME_H, contiguous crop copy
static uint8_t  *inkMaskBuf  = nullptr;  // FRAME_W*FRAME_H, ink mask inside whatever region is being scanned
static uint8_t  *packedBuf   = nullptr;  // (FRAME_W*FRAME_H)/8, final 1-bit packed output

// NEW: on-device digit classifier + its scratch glyph buffer. Tensor arena
// lives inside DigitClassifier and is internal-RAM (small, ~20KB), not PSRAM.
static DigitClassifier digitClassifier;
static uint8_t glyphScratch[MAX_GLYPH_W * MAX_GLYPH_H];  // MAX_GLYPH_W/H from digit_classifier_2.h

struct BBox { int x0, y0, x1, y1; uint32_t count; };

// ---------------------------------------------------------------------------
//  MODE / LOCK / SEGMENT / RECOGNIZE STATE
// ---------------------------------------------------------------------------
enum Mode { MODE_STREAM, MODE_SEARCHING, MODE_LOCKED };   // NEW: MODE_STREAM added
static Mode currentMode  = MODE_STREAM;   // NEW: boot into raw preview, not straight into searching
static bool segmentOn    = false;   // only acted on while MODE_LOCKED
static bool recognizeOn  = false;   // only acted on while MODE_LOCKED, independent of segmentOn
static uint8_t panelThresh = PANEL_THRESH;  // NEW: mutable copy, was previously a hardcoded #define
static bool showMask     = false;   // NEW: while SEARCHING/STREAM, send the panel mask instead of BW image

// Locked crop, full-frame coords. Valid only once currentMode == MODE_LOCKED.
static int lockedFx0 = 0, lockedFy0 = 0, lockedFx1 = 0, lockedFy1 = 0;

// Most recently *computed* crop from the auto-detector while searching.
// A LOCK command freezes onto whatever this was, so it always matches
// what the user last saw on the webpage.
static int  lastFx0 = 0, lastFy0 = 0, lastFx1 = 0, lastFy1 = 0;
static bool haveLastCrop = false;

// ---- inbound command protocol (sender -> cam): 0xC0 <cmd> ----
#define CMD_SYNC          0xC0
#define CMD_LOCK          0x01
#define CMD_UNLOCK        0x02   // also doubles as "start searching" from MODE_STREAM
#define CMD_SEGMENT_ON    0x03
#define CMD_SEGMENT_OFF   0x04
#define CMD_RECOGNIZE_ON  0x05
#define CMD_RECOGNIZE_OFF 0x06
#define CMD_STREAM         8    // NEW: back to raw full-frame preview from any mode
#define CMD_SET_PANEL_THRESH 9  // NEW: one value byte follows this command byte
#define CMD_SHOW_MASK_ON   10   // NEW: preview the post-threshold panel mask instead of BW crop
#define CMD_SHOW_MASK_OFF  11

static void pollCommands() {
  static bool sawSync = false;
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (!sawSync) {
      sawSync = (b == CMD_SYNC);
      continue;
    }
    sawSync = false;
    switch (b) {
      case CMD_LOCK:
        if (haveLastCrop) {
          lockedFx0 = lastFx0; lockedFy0 = lastFy0;
          lockedFx1 = lastFx1; lockedFy1 = lastFy1;
          currentMode = MODE_LOCKED;
          segmentOn = false;      // always start locked in plain mode
          recognizeOn = false;    // NEW: same rule for recognition
        }
        break;
      case CMD_UNLOCK:
        currentMode = MODE_SEARCHING;
        segmentOn = false;
        recognizeOn = false;      // NEW
        break;
      case CMD_SEGMENT_ON:
        if (currentMode == MODE_LOCKED) segmentOn = true;
        break;
      case CMD_SEGMENT_OFF:
        segmentOn = false;
        break;
      case CMD_RECOGNIZE_ON:      // NEW
        if (currentMode == MODE_LOCKED) recognizeOn = true;
        break;
      case CMD_RECOGNIZE_OFF:     // NEW
        recognizeOn = false;
        break;
      case CMD_STREAM:            // NEW: raw full-frame preview, from any mode
        currentMode = MODE_STREAM;
        segmentOn = false;
        recognizeOn = false;
        break;
      case CMD_SET_PANEL_THRESH: {   // NEW: value byte follows immediately behind this one
        uint32_t waitStart = millis();
        while (!Serial.available()) {
          if (millis() - waitStart > 200) return;   // value never arrived -- give up this cycle
        }
        panelThresh = Serial.read();
        break;
      }
      case CMD_SHOW_MASK_ON:      // NEW
        showMask = true;
        break;
      case CMD_SHOW_MASK_OFF:     // NEW
        showMask = false;
        break;
      default:
        break;   // unrecognized command byte -- ignore
    }
  }
}

// ---------------------------------------------------------------------------
//  Camera init
// ---------------------------------------------------------------------------
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk      = XCLK_GPIO_NUM;
  config.pin_pclk      = PCLK_GPIO_NUM;
  config.pin_vsync     = VSYNC_GPIO_NUM;
  config.pin_href      = HREF_GPIO_NUM;
  config.pin_sccb_sda  = SIOD_GPIO_NUM;
  config.pin_sccb_scl  = SIOC_GPIO_NUM;
  config.pin_pwdn      = PWDN_GPIO_NUM;
  config.pin_reset     = RESET_GPIO_NUM;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_GRAYSCALE;
  config.frame_size    = FRAME_SIZE;
  config.fb_location   = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.fb_count      = 1;
  config.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&config) != ESP_OK) {
    while (true) delay(1000);   // UART reserved for the binary protocol -- no Serial.print
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
}

// ---------------------------------------------------------------------------
//  Working-buffer allocation (PSRAM). Halts if allocation fails.
// ---------------------------------------------------------------------------
void initBuffers() {
  size_t n         = (size_t)FRAME_W * FRAME_H;
  size_t nIntegral = (size_t)(FRAME_W + 1) * (FRAME_H + 1);
  size_t nPacked   = (n + 7) / 8;

  maskBuf     = (uint8_t*) heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  maskBuf2    = (uint8_t*) heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  integralBuf = (uint32_t*)heap_caps_malloc(nIntegral * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
  visitedBuf  = (uint8_t*) heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  stackBuf    = (uint32_t*)heap_caps_malloc(n * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
  roiBuf      = (uint8_t*) heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  inkMaskBuf  = (uint8_t*) heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  packedBuf   = (uint8_t*) heap_caps_malloc(nPacked, MALLOC_CAP_SPIRAM);

  bool ok = maskBuf && maskBuf2 && integralBuf && visitedBuf &&
            stackBuf && roiBuf && inkMaskBuf && packedBuf;
  if (!ok) {
    while (true) delay(1000);   // out of PSRAM -- nothing we can do without UART
  }
}

// ---------------------------------------------------------------------------
//  Integral image over a tightly-packed w*h buffer. Stride is w+1.
// ---------------------------------------------------------------------------
static void buildIntegral(const uint8_t *img, int w, int h, uint32_t *integral) {
  int stride = w + 1;
  for (int x = 0; x <= w; x++) integral[x] = 0;
  for (int y = 1; y <= h; y++) {
    integral[y * stride] = 0;
    uint32_t rowSum = 0;
    for (int x = 1; x <= w; x++) {
      rowSum += img[(y - 1) * w + (x - 1)];
      integral[y * stride + x] = integral[(y - 1) * stride + x] + rowSum;
    }
  }
}

static inline uint32_t rectSum(const uint32_t *integral, int w, int x1, int y1, int x2, int y2) {
  int stride = w + 1;
  return integral[(y2 + 1) * stride + (x2 + 1)]
       - integral[y1 * stride + (x2 + 1)]
       - integral[(y2 + 1) * stride + x1]
       + integral[y1 * stride + x1];
}

static void thresholdBright(const uint8_t *gray, int w, int h, int thresh, uint8_t *mask) {
  int n = w * h;
  for (int i = 0; i < n; i++) mask[i] = (gray[i] >= thresh) ? 1 : 0;
}

static void dilateBinary(const uint8_t *mask, int w, int h, int radius, uint32_t *integral, uint8_t *out) {
  buildIntegral(mask, w, h, integral);
  for (int y = 0; y < h; y++) {
    int y1 = max(0, y - radius), y2 = min(h - 1, y + radius);
    for (int x = 0; x < w; x++) {
      int x1 = max(0, x - radius), x2 = min(w - 1, x + radius);
      out[y * w + x] = (rectSum(integral, w, x1, y1, x2, y2) > 0) ? 1 : 0;
    }
  }
}

static void erodeBinary(const uint8_t *mask, int w, int h, int radius, uint32_t *integral, uint8_t *out) {
  buildIntegral(mask, w, h, integral);
  for (int y = 0; y < h; y++) {
    int y1 = max(0, y - radius), y2 = min(h - 1, y + radius);
    for (int x = 0; x < w; x++) {
      int x1 = max(0, x - radius), x2 = min(w - 1, x + radius);
      uint32_t area = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);
      out[y * w + x] = (rectSum(integral, w, x1, y1, x2, y2) == area) ? 1 : 0;
    }
  }
}

static void floodFill(uint8_t *mask, int w, int h, uint8_t *visited, uint32_t *stack,
                       int startIdx, BBox *box) {
  int sp = 0;
  stack[sp++] = startIdx;
  visited[startIdx] = 1;
  int minX = startIdx % w, maxX = minX, minY = startIdx / w, maxY = minY;
  uint32_t count = 0;
  while (sp > 0) {
    int idx = stack[--sp];
    int x = idx % w, y = idx / w;
    count++;
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        if (dx == 0 && dy == 0) continue;
        int nx = x + dx, ny = y + dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        int nidx = ny * w + nx;
        if (mask[nidx] && !visited[nidx]) {
          visited[nidx] = 1;
          stack[sp++] = nidx;
        }
      }
    }
  }
  box->x0 = minX; box->y0 = minY; box->x1 = maxX + 1; box->y1 = maxY + 1; box->count = count;
}

static bool findLargestComponent(uint8_t *mask, int w, int h, uint8_t *visited,
                                  uint32_t *stack, BBox *largest) {
  memset(visited, 0, (size_t)w * h);
  BBox best = {0, 0, 0, 0, 0};
  bool found = false;
  int n = w * h;
  for (int i = 0; i < n; i++) {
    if (mask[i] && !visited[i]) {
      BBox b;
      floodFill(mask, w, h, visited, stack, i, &b);
      if (b.count > best.count) { best = b; found = true; }
    }
  }
  *largest = best;
  return found;
}

static float otsu1D(const int *heights, int n, int nbins = 32) {
  int minV = heights[0], maxV = heights[0];
  for (int i = 1; i < n; i++) {
    if (heights[i] < minV) minV = heights[i];
    if (heights[i] > maxV) maxV = heights[i];
  }
  if (minV == maxV) return (float)minV;

  float binWidth = (float)(maxV - minV) / nbins;
  static int hist[64];
  memset(hist, 0, sizeof(hist));
  for (int i = 0; i < n; i++) {
    int b = (int)((heights[i] - minV) / binWidth);
    if (b >= nbins) b = nbins - 1;
    hist[b]++;
  }

  float total = (float)n, sumAll = 0;
  for (int b = 0; b < nbins; b++) sumAll += hist[b] * (minV + (b + 0.5f) * binWidth);

  float sumB = 0, wB = 0, bestVar = -1, bestThresh = (float)minV;
  for (int b = 0; b < nbins; b++) {
    wB += hist[b];
    if (wB == 0) continue;
    float wF = total - wB;
    if (wF == 0) break;
    sumB += hist[b] * (minV + (b + 0.5f) * binWidth);
    float mB = sumB / wB, mF = (sumAll - sumB) / wF;
    float varBetween = wB * wF * (mB - mF) * (mB - mF);
    if (varBetween > bestVar) { bestVar = varBetween; bestThresh = minV + (b + 1) * binWidth; }
  }
  return bestThresh;
}

static inline void setPackedPixel(uint8_t *packed, int rowBytes, int w, int h, int x, int y, bool bit) {
  if (x < 0 || x >= w || y < 0 || y >= h) return;
  uint8_t m = (uint8_t)(1 << (7 - (x % 8)));
  int idx = y * rowBytes + (x / 8);
  if (bit) packed[idx] |= m; else packed[idx] &= (uint8_t)~m;
}

// [x0,y0,x1,y1) half-open, matching BBox convention from floodFill().
static void drawBoxOutline(uint8_t *packed, int rowBytes, int w, int h, int x0, int y0, int x1, int y1) {
  int xEnd = x1 - 1, yEnd = y1 - 1;
  if (xEnd < x0 || yEnd < y0) return;
  for (int x = x0; x <= xEnd; x++) {
    setPackedPixel(packed, rowBytes, w, h, x, y0,   false);
    setPackedPixel(packed, rowBytes, w, h, x, yEnd, false);
  }
  for (int y = y0; y <= yEnd; y++) {
    setPackedPixel(packed, rowBytes, w, h, x0,   y, false);
    setPackedPixel(packed, rowBytes, w, h, xEnd, y, false);
  }
}

// NEW: copies one box's pixels out of a strided crop buffer (stride =
// cropW) into a tightly-packed w*h scratch buffer the classifier can
// consume directly. Boxes bigger than the classifier's own MAX_GLYPH_W/H
// ceiling are rejected here -- with REGION_PADDING this shouldn't happen
// in practice, but we'd rather skip a glyph than overrun glyphScratch.
static bool extractGlyph(const uint8_t *crop, int cropW, const BBox &b,
                          uint8_t *dst, int *outW, int *outH) {
  int w = b.x1 - b.x0, h = b.y1 - b.y0;
  if (w <= 0 || h <= 0 || w > MAX_GLYPH_W || h > MAX_GLYPH_H) return false;
  for (int y = 0; y < h; y++) {
    memcpy(dst + y * w, crop + (b.y0 + y) * cropW + b.x0, w);
  }
  *outW = w; *outH = h;
  return true;
}

// ---- runs local-mean ink detection + connected components + Otsu height
//      split over an arbitrary w x h buffer (either the panel crop while
//      searching, or the locked crop while locked+segmented/recognized),
//      and returns up to MAX_COMPONENTS kept "digit-sized" blobs in
//      `outBoxes` (coordinates local to that same w x h buffer). ----


static int findDigitBlobs(uint8_t *srcBuf, int w, int h, BBox *outBoxes,
                           BBox *outCandidates = nullptr, int *outCandidateCount = nullptr) {
  buildIntegral(srcBuf, w, h, integralBuf);
  int halfBlock = BLOCK_SIZE / 2;
  for (int y = 0; y < h; y++) {
    int y1 = max(0, y - halfBlock), y2 = min(h - 1, y + halfBlock);
    for (int x = 0; x < w; x++) {
      int x1 = max(0, x - halfBlock), x2 = min(w - 1, x + halfBlock);
      uint32_t area = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);
      int localMean = (int)(rectSum(integralBuf, w, x1, y1, x2, y2) / area);
      int val = srcBuf[y * w + x];
      inkMaskBuf[y * w + x] = (val < (localMean - LOCAL_OFFSET)) ? 1 : 0;
    }
  }

  memset(visitedBuf, 0, (size_t)w * h);
  static BBox comps[MAX_COMPONENTS];
  int compCount = 0;
  int maxBarW = (int)(MAX_BAR_FRACTION * w);
  int maxBarH = (int)(MAX_BAR_FRACTION * h);
  int nPx = w * h;
  for (int i = 0; i < nPx; i++) {
    if (inkMaskBuf[i] && !visitedBuf[i]) {
      BBox b;
      floodFill(inkMaskBuf, w, h, visitedBuf, stackBuf, i, &b);
      if (b.count < 3) continue;
      int hgt = b.y1 - b.y0, wid = b.x1 - b.x0;
      if (hgt > maxBarH || wid > maxBarW) continue;
      if (compCount < MAX_COMPONENTS) comps[compCount++] = b;
    }
  }
  if (outCandidateCount) *outCandidateCount = 0;
  if (compCount < 2) return 0;

  static int heights[MAX_COMPONENTS];
  for (int i = 0; i < compCount; i++) heights[i] = comps[i].y1 - comps[i].y0;
  float cutoff = otsu1D(heights, compCount);

  // Digit-sized blobs (>= cutoff) go to outBoxes, same as before. Everything
  // that got rejected for being too short is now handed back separately via
  // outCandidates instead of being dropped -- it's the pool the decimal-point
  // check at the row-grouping stage draws from.
  int kept = 0, candCount = 0;
  for (int i = 0; i < compCount; i++) {
    if (heights[i] >= cutoff) {
      if (kept < MAX_COMPONENTS) outBoxes[kept++] = comps[i];
    } else if (outCandidates && candCount < MAX_COMPONENTS) {
      outCandidates[candCount++] = comps[i];
    }
  }
  if (outCandidateCount) *outCandidateCount = candCount;
  return kept;
}

// ---------------------------------------------------------------------------
//  Full pipeline. Bails out (skips the frame) wherever nothing plausible
//  is available -- SEARCHING mode can fail to find a panel/digits;
//  LOCKED mode only fails if it hasn't been given valid coordinates yet.
// ---------------------------------------------------------------------------

int fx0, fy0, fx1, fy1;   // final numbers-region crop, full-frame coords
static BBox digitBoxesFinal[MAX_COMPONENTS];   // CROP-LOCAL coords, ready to draw/classify as-is
int digitCount = 0;                            // >0 whenever segmentOn or recognizeOn found blobs
static BBox digitCandidatesFinal[MAX_COMPONENTS];  // NEW: sub-height-cutoff blobs -- decimal-point candidates
int candidateCount = 0;

void captureAndSend() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  if (fb->width != FRAME_W || fb->height != FRAME_H) {
    esp_camera_fb_return(fb);
    return;
  }

  int fx0, fy0, fx1, fy1;   // final numbers-region crop, full-frame coords
  static BBox digitBoxesFinal[MAX_COMPONENTS];   // CROP-LOCAL coords, ready to draw/classify as-is
  int digitCount = 0;                            // >0 whenever segmentOn or recognizeOn found blobs

  if (currentMode == MODE_STREAM) {
    // ---- MODE_STREAM: no detection at all -- just the whole raw frame,
    //      BW-thresholded and packed the same way every other mode's crop
    //      is, so this reuses the existing wire format/webpage renderer
    //      unchanged. Purely a "what is the cam actually looking at"
    //      preview so you can position/frame the panel before Start
    //      Searching kicks off the auto-detect pipeline. ----
    fx0 = 0; fy0 = 0; fx1 = FRAME_W; fy1 = FRAME_H;

  } else if (currentMode == MODE_SEARCHING) {
    // ---- STAGE 1: detect the panel (largest bright blob) ----
    thresholdBright(fb->buf, FRAME_W, FRAME_H, panelThresh, maskBuf);
    dilateBinary(maskBuf,  FRAME_W, FRAME_H, MORPH_RADIUS, integralBuf, maskBuf2);
    erodeBinary (maskBuf2, FRAME_W, FRAME_H, MORPH_RADIUS, integralBuf, maskBuf);
    erodeBinary (maskBuf,  FRAME_W, FRAME_H, MORPH_RADIUS, integralBuf, maskBuf2);
    dilateBinary(maskBuf2, FRAME_W, FRAME_H, MORPH_RADIUS, integralBuf, maskBuf);

    if (showMask) {   // NEW: bypass the crop pipeline entirely, show exactly what panelThresh finds right now
      int rowBytes   = (FRAME_W + 7) / 8;
      int packedSize = rowBytes * FRAME_H;
      memset(packedBuf, 0, packedSize);
      for (int y = 0; y < FRAME_H; y++) {
        for (int x = 0; x < FRAME_W; x++) {
          if (maskBuf[y * FRAME_W + x]) packedBuf[y * rowBytes + (x / 8)] |= (1 << (7 - (x % 8)));
        }
      }
      esp_camera_fb_return(fb);
      Serial.write(0xAA); Serial.write(0x55);
      Serial.write((uint8_t)0x10);              // NEW status bit4 = mask preview frame
      Serial.write(panelThresh);
      Serial.write((uint8_t)(FRAME_W >> 8)); Serial.write((uint8_t)(FRAME_W & 0xFF));
      Serial.write((uint8_t)(FRAME_H >> 8)); Serial.write((uint8_t)(FRAME_H & 0xFF));
      Serial.write(packedBuf, packedSize);
      Serial.write((uint8_t)0);                 // textLen = 0
      Serial.flush();
      return;
    }

    BBox panel;
    if (!findLargestComponent(maskBuf, FRAME_W, FRAME_H, visitedBuf, stackBuf, &panel)) {
      esp_camera_fb_return(fb);
      return;
    }
    int panelW = panel.x1 - panel.x0;
    int panelH = panel.y1 - panel.y0;
    if (panelW <= 0 || panelH <= 0) { esp_camera_fb_return(fb); return; }

    // ---- STAGE 2: copy panel crop into roiBuf ----
    for (int y = 0; y < panelH; y++) {
      memcpy(roiBuf + y * panelW, fb->buf + (panel.y0 + y) * FRAME_W + panel.x0, panelW);
    }

    // ---- STAGE 3-5: ink blobs inside the panel, digit-sized subset ----
    static BBox panelDigitBoxes[MAX_COMPONENTS];
    int panelDigitCount = findDigitBlobs(roiBuf, panelW, panelH, panelDigitBoxes);
    if (panelDigitCount == 0) { esp_camera_fb_return(fb); return; }

    int dx0 = INT32_MAX, dy0 = INT32_MAX, dx1 = INT32_MIN, dy1 = INT32_MIN;
    for (int i = 0; i < panelDigitCount; i++) {
      if (panelDigitBoxes[i].x0 < dx0) dx0 = panelDigitBoxes[i].x0;
      if (panelDigitBoxes[i].y0 < dy0) dy0 = panelDigitBoxes[i].y0;
      if (panelDigitBoxes[i].x1 > dx1) dx1 = panelDigitBoxes[i].x1;
      if (panelDigitBoxes[i].y1 > dy1) dy1 = panelDigitBoxes[i].y1;
    }

    // ---- STAGE 6: numbers-region bbox -> full-frame coords, padded & clamped ----
    fx0 = panel.x0 + dx0 - REGION_PADDING;
    fy0 = panel.y0 + dy0 - REGION_PADDING;
    fx1 = panel.x0 + dx1 + REGION_PADDING;
    fy1 = panel.y0 + dy1 + REGION_PADDING;
    if (fx0 < 0) fx0 = 0;
    if (fy0 < 0) fy0 = 0;
    if (fx1 > FRAME_W) fx1 = FRAME_W;
    if (fy1 > FRAME_H) fy1 = FRAME_H;
    if (fx1 - fx0 <= 0 || fy1 - fy0 <= 0) { esp_camera_fb_return(fb); return; }

    // Remember this so a LOCK command freezes onto exactly what was just computed.
    lastFx0 = fx0; lastFy0 = fy0; lastFx1 = fx1; lastFy1 = fy1;
    haveLastCrop = true;

    // SEARCHING never draws outlines or recognizes -- plain BW only.
    digitCount = 0;

  } else {
    // ---- MODE_LOCKED: skip detection, use the frozen rectangle ----
    fx0 = lockedFx0; fy0 = lockedFy0; fx1 = lockedFx1; fy1 = lockedFy1;
    if (fx1 - fx0 <= 0 || fy1 - fy0 <= 0) { esp_camera_fb_return(fb); return; }
  }

  int cropW = fx1 - fx0, cropH = fy1 - fy0;

  // ---- STAGE 7: extract the numbers-region crop (both modes) ----
  for (int y = 0; y < cropH; y++) {
    memcpy(roiBuf + y * cropW, fb->buf + (fy0 + y) * FRAME_W + fx0, cropW);
  }
  esp_camera_fb_return(fb);   // done with the raw frame, free it early

  // ---- Locked + (segmentation and/or recognition): find digit blobs
  //      *within this crop only*. Both features need the same blob list,
  //      so compute it once whenever either is requested. ----
  bool needBlobs = (currentMode == MODE_LOCKED) && (segmentOn || recognizeOn);
  if (needBlobs) {
    digitCount = findDigitBlobs(roiBuf, cropW, cropH, digitBoxesFinal,
                                 recognizeOn ? digitCandidatesFinal : nullptr,
                                 recognizeOn ? &candidateCount : nullptr);
    // digitBoxesFinal[] is already crop-local -- no offset translation needed,
    // since the crop *is* the reference frame in this mode (no panel).
  }

  // ---- STAGE 8: local adaptive threshold + 1-bit pack (both modes) ----
  buildIntegral(roiBuf, cropW, cropH, integralBuf);
  int rowBytes   = (cropW + 7) / 8;
  int packedSize = rowBytes * cropH;
  memset(packedBuf, 0, packedSize);
  int halfBlockBW = BW_BLOCK_SIZE / 2;
  for (int y = 0; y < cropH; y++) {
    int y1 = max(0, y - halfBlockBW), y2 = min(cropH - 1, y + halfBlockBW);
    for (int x = 0; x < cropW; x++) {
      int x1 = max(0, x - halfBlockBW), x2 = min(cropW - 1, x + halfBlockBW);
      uint32_t area = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);
      int localThresh = (int)(rectSum(integralBuf, cropW, x1, y1, x2, y2) / area) - BW_OFFSET;
      if ((int)roiBuf[y * cropW + x] >= localThresh) {
        packedBuf[y * rowBytes + (x / 8)] |= (1 << (7 - (x % 8)));
      }
    }
  }

  // ---- STAGE 8.5: outline every kept digit blob (segmentOn only -- if
  //      only recognizeOn is set, digitBoxesFinal[] is still needed below
  //      for classification, but the picture itself stays plain) ----
  if (segmentOn) {
    for (int i = 0; i < digitCount; i++) {
      BBox &b = digitBoxesFinal[i];
      drawBoxOutline(packedBuf, rowBytes, cropW, cropH, b.x0, b.y0, b.x1, b.y1);
    }
  }

  // ---- STAGE 8.6: recognize -- group the kept blobs into rows by vertical
  //      position (a panel can have several stacked numeric rows, and a
  //      flat left-to-right sort across the whole crop interleaves them
  //      into one meaningless line), sort each row left-to-right, classify,
  //      and join rows with '\n'. That's what "the exact same order they
  //      were detected in" means once there's more than one row: row order
  //      top-to-bottom, then reading order left-to-right within a row --
  //      findDigitBlobs()/flood-fill don't return boxes in any spatial
  //      order on their own. ----
static char recognizedText[MAX_RECOGNIZED_CHARS + 1];
  int recognizedLen = 0;
  if (currentMode == MODE_LOCKED && recognizeOn && digitCount > 0) {
    // 1. Sort digit blobs by vertical center -- discovers row order
    //    (top row first) without touching x at all.
    static int order[MAX_COMPONENTS];
    for (int i = 0; i < digitCount; i++) order[i] = i;
    for (int i = 1; i < digitCount; i++) {
      int keyIdx = order[i];
      float keyYc = 0.5f * (digitBoxesFinal[keyIdx].y0 + digitBoxesFinal[keyIdx].y1);
      int j = i - 1;
      while (j >= 0) {
        int cmpIdx = order[j];
        float cmpYc = 0.5f * (digitBoxesFinal[cmpIdx].y0 + digitBoxesFinal[cmpIdx].y1);
        if (cmpYc <= keyYc) break;
        order[j + 1] = order[j];
        j--;
      }
      order[j + 1] = keyIdx;
    }

    // 2. Average glyph height sets the row-gap threshold.
    long heightSum = 0;
    for (int i = 0; i < digitCount; i++) heightSum += (digitBoxesFinal[i].y1 - digitBoxesFinal[i].y0);
    float avgHeight = (float)heightSum / digitCount;
    float rowGapThresh = avgHeight * ROW_GAP_FRACTION;

    static bool candidateUsed[MAX_COMPONENTS];
    for (int i = 0; i < candidateCount; i++) candidateUsed[i] = false;

    // Combined per-row sequence: either a real digit (index into
    // digitBoxesFinal) or a decimal point (index into digitCandidatesFinal),
    // tagged and x0-sorted together.
    struct RowEntry { int idx; bool isDot; int x0; };
    static RowEntry rowEntries[MAX_COMPONENTS];

    int runStart = 0;
    float prevYc = 0.5f * (digitBoxesFinal[order[0]].y0 + digitBoxesFinal[order[0]].y1);

    for (int i = 1; i <= digitCount; i++) {
      bool endOfRun = (i == digitCount);
      float yc = 0.0f;
      if (!endOfRun) yc = 0.5f * (digitBoxesFinal[order[i]].y0 + digitBoxesFinal[order[i]].y1);

      if (endOfRun || (yc - prevYc) > rowGapThresh) {
        // ---- this row is digitBoxesFinal[order[runStart..i-1]] ----

        // Row's vertical band + baseline, from its real digits only.
        int rowY0 = INT32_MAX, rowBaseline = INT32_MIN;
        long rowHeightSum = 0;
        for (int a = runStart; a < i; a++) {
          BBox &b = digitBoxesFinal[order[a]];
          if (b.y0 < rowY0) rowY0 = b.y0;
          if (b.y1 > rowBaseline) rowBaseline = b.y1;
          rowHeightSum += (b.y1 - b.y0);
        }
        float rowAvgHeight = (float)rowHeightSum / (i - runStart);
        float rowMargin    = DOT_ROW_MARGIN_FRACTION * rowAvgHeight;
        float maxDotSize   = DOT_MAX_SIZE_FRACTION   * rowAvgHeight;
        float baselineTol  = DOT_BASELINE_TOLERANCE  * rowAvgHeight;

        // Build the combined (digits + any matching dots) entry list for this row.
        int entryCount = 0;
        for (int a = runStart; a < i; a++) {
          BBox &b = digitBoxesFinal[order[a]];
          rowEntries[entryCount++] = { order[a], false, b.x0 };
        }
        for (int c = 0; c < candidateCount; c++) {
          if (candidateUsed[c]) continue;
          BBox &cb = digitCandidatesFinal[c];
          float cYc = 0.5f * (cb.y0 + cb.y1);
          if (cYc < rowY0 - rowMargin || cYc > rowBaseline + rowMargin) continue;  // wrong row
          int cw = cb.x1 - cb.x0, ch = cb.y1 - cb.y0;
          if (cw > maxDotSize || ch > maxDotSize) continue;                        // too big for a dot
          if (fabsf((float)cb.y1 - rowBaseline) > baselineTol) continue;           // not on the baseline
          candidateUsed[c] = true;
          if (entryCount < MAX_COMPONENTS) rowEntries[entryCount++] = { c, true, cb.x0 };
        }

        // Sort the combined row left-to-right by x0.
        for (int a = 1; a < entryCount; a++) {
          RowEntry key = rowEntries[a];
          int b = a - 1;
          while (b >= 0 && rowEntries[b].x0 > key.x0) {
            rowEntries[b + 1] = rowEntries[b];
            b--;
          }
          rowEntries[b + 1] = key;
        }

        // Emit: classify real digits, hardcode dots as '.'.
        for (int a = 0; a < entryCount && recognizedLen < MAX_RECOGNIZED_CHARS; a++) {
          if (rowEntries[a].isDot) {
            recognizedText[recognizedLen++] = '.';
            continue;
          }
          int gw, gh;
          BBox &b2 = digitBoxesFinal[rowEntries[a].idx];
          if (extractGlyph(roiBuf, cropW, b2, glyphScratch, &gw, &gh)) {
            int cls = digitClassifier.classify(glyphScratch, gw, gh);
            recognizedText[recognizedLen++] = (cls >= 0) ? digitClassifier.classChar(cls) : '?';
          } else {
            recognizedText[recognizedLen++] = '?';
          }
        }

        if (!endOfRun && recognizedLen < MAX_RECOGNIZED_CHARS) {
          recognizedText[recognizedLen++] = '\n';
        }
        runStart = i;
      }
      if (!endOfRun) prevYc = yc;
    }
  }
  recognizedText[recognizedLen] = '\0';

  // ---- STAGE 9: status byte + send over UART ----
  //      0xAA 0x55 | status(1) | w(2) | h(2) | packed data | textLen(1) | text(textLen)
  uint8_t statusByte = 0;
  if (currentMode == MODE_LOCKED) statusByte |= 0x01;
  if (currentMode == MODE_LOCKED && segmentOn   && digitCount    > 0) statusByte |= 0x02;
  if (currentMode == MODE_LOCKED && recognizeOn && recognizedLen > 0) statusByte |= 0x04;
  if (currentMode == MODE_STREAM) statusByte |= 0x08;   // NEW: raw full-frame preview

  Serial.write(0xAA); Serial.write(0x55);
  Serial.write(statusByte);
  Serial.write(panelThresh);
  Serial.write((uint8_t)(cropW >> 8)); Serial.write((uint8_t)(cropW & 0xFF));
  Serial.write((uint8_t)(cropH >> 8)); Serial.write((uint8_t)(cropH & 0xFF));
  Serial.write(packedBuf, packedSize);
  Serial.write((uint8_t)recognizedLen);   // NEW: always sent, 0 when not recognizing
  if (recognizedLen > 0) Serial.write((const uint8_t*)recognizedText, recognizedLen);
  Serial.flush();
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  initCamera();
  initBuffers();
  if (!digitClassifier.begin()) {
    while (true) delay(1000);   // model/schema mismatch or arena alloc failure -- UART reserved, can't report
  }
  delay(200);
  // From this point on, UART0 carries the binary frame protocol (TX) and
  // the tiny command protocol (RX).
}

void loop() {
  pollCommands();     // apply any LOCK/UNLOCK/SEGMENT_*/RECOGNIZE_* that arrived
  captureAndSend();
  delay(800);         // snapshot interval — adjust as needed
}
