// Step 3: blood pressure display reader for the AI-Thinker ESP32-CAM.
//
// Reads SYS / DIA / PULSE off the monitor's seven-segment display and sends
// each new result to SEND_URL. Work through settings 1 to 5 below, upload,
// and open the address printed in the Serial Monitor to check it reads right.
//
//   http://<ip>/            live view with the digit boxes, plus status
//   http://<ip>/img?raw=1   raw frame, for the digit mapper if the camera moves
//   http://<ip>/debug       per-digit segment detail
//   http://<ip>/read        read the display right now, as JSON
//
// Board: "AI Thinker ESP32-CAM", PSRAM enabled. For OTA uploads pick a
// partition scheme with an OTA slot, e.g. "Minimal SPIFFS (1.9MB APP with OTA)".

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include "driver/rtc_io.h"

// Anything defined in secrets.h (git-ignored) wins over the placeholders below.
#if __has_include("secrets.h")
#include "secrets.h"
#endif

// ================ 1. WiFi ================

#ifndef WIFI_SSID
#define WIFI_SSID "your-wifi-name"
#define WIFI_PASS "your-wifi-password"
#endif

// A fixed address makes reconnecting after sleep quicker. Optional.
const bool  USE_STATIC_IP = false;
IPAddress   STATIC_IP (192, 168, 1, 60);
IPAddress   GATEWAY   (192, 168, 1, 1);
IPAddress   SUBNET    (255, 255, 255, 0);
IPAddress   DNS_SERVER(192, 168, 1, 1);

// ================ 2. Where readings go ================
// Each reading is POSTed as JSON:
//   {"user":"Me","systolic":121,"diastolic":67,"pulse":52}
// A Home Assistant webhook, Node-RED or your own server all take this as is.
// To send it some other way, change sendReading().

#ifndef SEND_URL
#define SEND_URL  "http://192.168.1.10:8123/api/webhook/blood-pressure"
#endif
#ifndef SEND_AUTH
#define SEND_AUTH ""   // sent as the Authorization header if set, e.g. "Bearer abc123"
#endif

// ================ 3. Who is measuring ================
// No buttons: every reading goes out as USER_NAMES[0].
// With buttons (GPIO to GND): press A or B during the measurement, or up to
// USER_WAIT_MS after the result, to say whose reading it is.

const char* USER_NAMES[2] = { "Me", "Partner" };
const int   BTN_A = -1;   // e.g. 13, or -1 for no button
const int   BTN_B = -1;   // e.g. 15, or -1 for no button

// ================ 4. Sleep between readings (optional) ================
// -1: the board stays awake and on WiFi all the time. Simplest.
// A pin (e.g. 14) wired to the monitor's START button line: the board
// deep-sleeps, wakes when START is pressed, sends the result and sleeps again.
// Its web page and OTA then only run for a while after power-on, or after
// holding A and B together for 2 seconds.

const int   WAKE_PIN         = -1;
const bool  WAKE_ACTIVE_HIGH = false;  // false: the START line idles high and pressing START pulls it low

// ================ 5. Digit map ================
// Made in step 2. Until then CALIBRATED is false and the board only serves its
// live view. One box per digit, {x, y, width, height} on the 320x240 frame.
// The number after each name is that row's threshold: a segment counts as lit
// when it is that % darker than the background inside the digit.

struct Box { int x, y, w, h; };
struct Field { const char* name; int pct; int nDigits; Box digits[3]; };

// A reading waiting to be sent. Declared up here because the Arduino IDE adds
// function declarations above the first function, and they need this type.
struct Queued { uint8_t user; uint16_t sys, dia, pul; };

// ---- BEGIN DIGIT MAP (made by the digit mapper, replace the whole block) ----
const bool  CALIBRATED    = false;
const bool  ROTATE_180    = false;
const bool  DARK_ON_LIGHT = true;
const float SLANT         = 0.08;
const int   SEARCH_PX     = 2;
const int   GUARD_PCT     = 2;
const int   MIN_THRESH    = 10;

Field fields[] = {
  {"sys", 25, 3, {{131,  32, 29, 52}, {162,  32, 29, 52}, {193,  32, 29, 52}}},
  {"dia", 25, 3, {{130,  89, 29, 56}, {163,  89, 29, 56}, {196,  89, 29, 56}}},
  {"pul", 20, 3, {{164, 152, 19, 39}, {186, 152, 19, 39}, {208, 152, 19, 39}}},
};
// ---- END DIGIT MAP ----
const int NUM_FIELDS = sizeof(fields) / sizeof(fields[0]);

// ================ Advanced ================

#ifndef OTA_PASS
#define OTA_PASS "change-me"   // the IDE asks for this when uploading over WiFi
#endif
const char* HOSTNAME = "bp-reader";    // http://bp-reader.local, and the OTA port name

const int   AEC_VALUE     = 300;   // starting exposure, 0-1200
const int   AGC_GAIN      = 2;     // starting gain, 0-30
const bool  AUTO_EXPOSURE = true;  // keep adjusting exposure so the LCD background stays near TARGET_BG
const int   TARGET_BG     = 130;   // LCD background brightness to aim for (0-255)
const int   BG_BAND       = 25;    // leave exposure alone while the background is within this of the target
const bool  USE_FLASH     = false; // onboard LED on GPIO 4 (expect glare if the camera is square-on)

const int   POLL_MS       = 1000;  // how often the display is checked
const int   RETRY_MS      = 250;   // how soon it looks again while exposure is still settling
const int   STABLE_POLLS  = 3;     // identical polls needed before a reading counts
const int   CLEAR_POLLS   = 5;     // unreadable polls before it is ready for the next reading

const unsigned long DEBOUNCE_MS     = 50;
const int   STATUS_LED_PIN          = 33;      // red LED on the back of the board, lit when LOW. -1 for none.
const unsigned long USER_WAIT_MS    = 60000;   // after a result, how long A or B can still be pressed
const unsigned long BLANK_SLEEP_MS  = 15000;   // sleep mode: sleep if the display shows nothing for this long
const unsigned long AWAKE_MAX_MS    = 300000;  // sleep mode: never stay awake longer than this after START
const unsigned long MAINT_HOLD_MS   = 2000;    // hold A and B together this long for the web page and OTA
const unsigned long MAINT_IDLE_MS   = 300000;  // sleep mode: web page and OTA end this long after last use
const unsigned long WIFI_TIMEOUT_MS = 10000;   // give up connecting after this (the reading stays queued)
const int   HTTP_TIMEOUT_MS         = 5000;

// Sample points inside a digit box, as percentages of box width/height.
// Order: a (top), b (top right), c (bottom right), d (bottom),
//        e (bottom left), f (top left), g (middle).
const int SEG_PTS[7][2] = {
  {50,  8}, {85, 28}, {85, 72}, {50, 92}, {15, 72}, {15, 28}, {50, 50}
};
const bool SEG_HORIZ[7] = { true, false, false, true, false, false, true };

// The two "holes" of the 8 are never lit, so they give the background level.
// Three points across each hole: the brightest is used, so a segment's shadow
// or edge falling on one of them is ignored.
const int HOLE_X[3] = { 35, 50, 65 };
const int HOLE_Y[2] = { 28, 72 };          // top hole, bottom hole
// Which hole each segment is compared with: 0 top, 1 bottom, 2 the average of both.
const int SEG_HOLE[7] = { 0, 0, 1, 1, 1, 0, 2 };

bool sleepMode()  { return WAKE_PIN >= 0 && CALIBRATED; }
bool hasButtons() { return BTN_A >= 0 || BTN_B >= 0; }

// ---------------- Camera ----------------

const int IMG_W = 320;
const int IMG_H = 240;
const int FLASH_PIN = 4;
const int CAM_PWDN_PIN = 32;

WebServer server(80);   // only runs in maintenance mode

// Kept in RTC memory, so they survive deep sleep (and reset on power-up):
// the exposure in use and how far the digits sit from their calibrated place.
RTC_DATA_ATTR int aecNow  = AEC_VALUE;
RTC_DATA_ATTR int gainNow = AGC_GAIN;
RTC_DATA_ATTR int alignX  = 0;
RTC_DATA_ATTR int alignY  = 0;

bool cameraOn = false;

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = 5;   c.pin_d1 = 18;  c.pin_d2 = 19;  c.pin_d3 = 21;
  c.pin_d4 = 36;  c.pin_d5 = 39;  c.pin_d6 = 34;  c.pin_d7 = 35;
  c.pin_xclk = 0; c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sccb_sda = 26; c.pin_sccb_scl = 27;
  c.pin_pwdn = CAM_PWDN_PIN; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size   = FRAMESIZE_QVGA;       // 320x240
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&c) != ESP_OK) return false;
  cameraOn = true;

  sensor_t* s = esp_camera_sensor_get();
  if (CALIBRATED) {
    // Take exposure and gain off automatic. The reader steers them itself
    // from the LCD background (see adjustExposure).
    s->set_exposure_ctrl(s, 0);
    s->set_aec2(s, 0);
    s->set_aec_value(s, aecNow);    // last exposure used before sleeping, or AEC_VALUE after power-on
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, gainNow);
  }
  // Not calibrated: the sensor's own auto exposure stays on, so the setup
  // image is sensibly exposed before the reader knows where the LCD is.
  s->set_whitebal(s, 0);
  return true;
}

// Powers the camera down, e.g. before sending (WiFi and the camera together
// can brown the board out) and before sleeping.
void cameraOff() {
  if (!cameraOn) return;
  esp_camera_deinit();
  cameraOn = false;
  pinMode(CAM_PWDN_PIN, OUTPUT);
  digitalWrite(CAM_PWDN_PIN, HIGH);
}

// Returns a fresh frame, turned the right way up unless rotate is false.
// Caller must esp_camera_fb_return() it.
camera_fb_t* grabFrame(bool rotate) {
  if (!cameraOn) return nullptr;
  if (USE_FLASH) { digitalWrite(FLASH_PIN, HIGH); delay(150); }
  camera_fb_t* fb = esp_camera_fb_get();   // first one may be stale, bin it
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (USE_FLASH) digitalWrite(FLASH_PIN, LOW);
  if (fb && rotate && ROTATE_180) {
    // Reversing the pixel order of a greyscale frame is a 180 degree rotation.
    size_t n = (size_t)IMG_W * IMG_H;
    if (fb->len < n) n = fb->len;
    for (size_t i = 0, j = n - 1; i < j; i++, j--) {
      uint8_t t = fb->buf[i];
      fb->buf[i] = fb->buf[j];
      fb->buf[j] = t;
    }
  }
  return fb;
}

// ---------------- Decoding ----------------


// Sample point position, with the slant correction (rounded to the nearest
// pixel) and the current alignment applied.
int ptX(const Box& b, int px, int py) {
  return alignX + b.x + b.w * px / 100 + (int)lroundf(SLANT * b.h * (50 - py) / 100.0f);
}
int ptY(const Box& b, int py) {
  return alignY + b.y + b.h * py / 100;
}

int avgAt(const uint8_t* img, int cx, int cy) {
  int sum = 0, n = 0;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int x = cx + dx, y = cy + dy;
      if (x < 0 || y < 0 || x >= IMG_W || y >= IMG_H) continue;
      sum += img[y * IMG_W + x];
      n++;
    }
  }
  return n ? sum / n : 0;
}

int sampleAt(const uint8_t* img, const Box& b, int px, int py) {
  return avgAt(img, ptX(b, px, py), ptY(b, py));
}

// Mean of a 5 pixel line centred on (cx, cy), running along a segment.
int lineAt(const uint8_t* img, int cx, int cy, bool horiz) {
  int sum = 0, n = 0;
  for (int k = -2; k <= 2; k++) {
    int x = horiz ? cx + k : cx;
    int y = horiz ? cy : cy + k;
    if (x < 0 || y < 0 || x >= IMG_W || y >= IMG_H) continue;
    sum += img[y * IMG_W + x];
    n++;
  }
  return n ? sum / n : 0;
}

// Level of segment i: the darkest line found within SEARCH_PX either side of
// where the segment should be, so a small camera shift does not lose it.
int segmentLevel(const uint8_t* img, const Box& b, int i) {
  int cx = ptX(b, SEG_PTS[i][0], SEG_PTS[i][1]);
  int cy = ptY(b, SEG_PTS[i][1]);
  int best = -1;
  for (int o = -SEARCH_PX; o <= SEARCH_PX; o++) {
    int v = SEG_HORIZ[i] ? lineAt(img, cx, cy + o, true) : lineAt(img, cx + o, cy, false);
    if (best < 0 || (DARK_ON_LIGHT ? v < best : v > best)) best = v;
  }
  return best;
}

// Background level in one hole of the 8 (0 top, 1 bottom).
int holeLevel(const uint8_t* img, const Box& b, int hole) {
  int best = -1;
  for (int k = 0; k < 3; k++) {
    int v = sampleAt(img, b, HOLE_X[k], HOLE_Y[hole]);
    if (best < 0 || (DARK_ON_LIGHT ? v > best : v < best)) best = v;
  }
  return best;
}

// Fills diff[7] with how far each segment stands out from its background and
// ref[7] with that background level (order a..g, higher diff = more clearly lit).
void segmentDiffs(const uint8_t* img, const Box& b, int diff[7], int ref[7]) {
  int hole[3];
  hole[0] = holeLevel(img, b, 0);
  hole[1] = holeLevel(img, b, 1);
  hole[2] = (hole[0] + hole[1]) / 2;
  for (int i = 0; i < 7; i++) {
    int v = segmentLevel(img, b, i);
    ref[i]  = hole[SEG_HOLE[i]];
    diff[i] = DARK_ON_LIGHT ? (ref[i] - v) : (v - ref[i]);
  }
}

const int SEG_OFF    = 0;
const int SEG_ON     = 1;
const int SEG_UNSURE = 2;

// Decides one segment from its contrast, its background level and the row's threshold.
int classify(int diff, int ref, int pct) {
  if (ref * pct / 100 < MIN_THRESH) return SEG_UNSURE;    // frame too dark to judge
  if (diff * 100 > ref * (pct + GUARD_PCT)) return SEG_ON;
  if (diff * 100 < ref * (pct - GUARD_PCT)) return SEG_OFF;
  return SEG_UNSURE;
}

// Returns the 7-bit pattern (bit 6 = a ... bit 0 = g), or -1 if any segment was unsure.
int segmentMask(const uint8_t* img, const Box& b, int pct) {
  int diff[7], ref[7];
  segmentDiffs(img, b, diff, ref);
  int mask = 0;
  for (int i = 0; i < 7; i++) {
    int c = classify(diff[i], ref[i], pct);
    if (c == SEG_UNSURE) return -1;
    mask = (mask << 1) | (c == SEG_ON ? 1 : 0);
  }
  return mask;
}

bool frameMisaligned = false;   // set when the digits in the current frame are off their boxes
int  litSegments = 0;           // clearly lit segments found in the latest frame

// ---------------- Alignment ----------------

// In every frame that shows digits, each clearly lit segment is looked for a
// few pixels either side of where it should be, and the centre of its dark
// band is measured. If they are consistently off to one side, the camera has
// shifted slightly and all the boxes follow it. If one row disagrees with the
// rest (the camera has turned, or something is in the way), the frame is
// rejected. A move bigger than ALIGN_LIMIT needs a recalibration.
const int ALIGN_LIMIT = 10;

int medianOf(int* v, int n) {
  for (int i = 1; i < n; i++) {
    int t = v[i], j = i;
    while (j > 0 && v[j - 1] > t) { v[j] = v[j - 1]; j--; }
    v[j] = t;
  }
  return v[n / 2];
}

// Measures how far segment i of box b sits from its expected place, in tenths
// of a pixel. Returns false if the segment is not clearly lit.
bool segmentOffset(const uint8_t* img, const Box& b, int i, int ref, int pct, int* tenths) {
  int reach = b.w / 6;                     // search reach scales with digit size
  if (reach < 3) reach = 3;
  if (reach > 6) reach = 6;
  int cx = ptX(b, SEG_PTS[i][0], SEG_PTS[i][1]);
  int cy = ptY(b, SEG_PTS[i][1]);
  int prof[13];
  int dmax = 0;
  for (int o = -reach; o <= reach; o++) {
    int v = SEG_HORIZ[i] ? lineAt(img, cx, cy + o, true) : lineAt(img, cx + o, cy, false);
    int d = DARK_ON_LIGHT ? (ref - v) : (v - ref);
    prof[o + reach] = d;
    if (d > dmax) dmax = d;
  }
  if (ref <= 0 || dmax * 200 < ref * pct * 3) return false;   // needs 1.5x the row threshold
  long sw = 0, sow = 0;
  for (int o = -reach; o <= reach; o++) {
    int w = prof[o + reach] - dmax / 2;     // centre of the darker half of the band
    if (w > 0) { sw += w; sow += (long)w * o; }
  }
  if (sw == 0) return false;
  *tenths = (int)(sow * 10 / sw);
  return true;
}

// Rounds tenths of a pixel to whole pixels.
int tenthsToPx(int t) {
  return t >= 0 ? (t + 5) / 10 : -((-t + 5) / 10);
}

// Updates alignX/alignY from this frame. Returns false if the rows disagree
// about where the digits are, in which case the frame should not be read.
bool updateAlignment(const uint8_t* img) {
  int allX[NUM_FIELDS * 3 * 4], allY[NUM_FIELDS * 3 * 3];
  int rowX[NUM_FIELDS][3 * 4], rowY[NUM_FIELDS][3 * 3];
  int nRowX[NUM_FIELDS], nRowY[NUM_FIELDS];
  int nx = 0, ny = 0;
  for (int f = 0; f < NUM_FIELDS; f++) {
    nRowX[f] = 0;
    nRowY[f] = 0;
    for (int d = 0; d < fields[f].nDigits; d++) {
      const Box& b = fields[f].digits[d];
      int hole[3];
      hole[0] = holeLevel(img, b, 0);
      hole[1] = holeLevel(img, b, 1);
      hole[2] = (hole[0] + hole[1]) / 2;
      for (int i = 0; i < 7; i++) {
        int t;
        if (!segmentOffset(img, b, i, hole[SEG_HOLE[i]], fields[f].pct, &t)) continue;
        if (SEG_HORIZ[i]) { allY[ny++] = t; rowY[f][nRowY[f]++] = t; }
        else              { allX[nx++] = t; rowX[f][nRowX[f]++] = t; }
      }
    }
  }
  litSegments = nx + ny;
  // Follow the digits once they are 1.5px or more out. Smaller wobble is left
  // to the per-segment search.
  int moveX = 0, moveY = 0;
  if (nx >= 4) { int m = medianOf(allX, nx); if (m >= 15 || m <= -15) moveX = tenthsToPx(m); }
  if (ny >= 3) { int m = medianOf(allY, ny); if (m >= 15 || m <= -15) moveY = tenthsToPx(m); }
  alignX += moveX;
  alignY += moveY;
  // After the move, every row with enough lit segments should be within 2px.
  for (int f = 0; f < NUM_FIELDS; f++) {
    if (nRowX[f] >= 3) {
      int r = medianOf(rowX[f], nRowX[f]) - moveX * 10;
      if (r >= 20 || r <= -20) return false;
    }
    if (nRowY[f] >= 3) {
      int r = medianOf(rowY[f], nRowY[f]) - moveY * 10;
      if (r >= 20 || r <= -20) return false;
    }
  }
  return true;
}

bool alignmentLost() {
  return alignX > ALIGN_LIMIT || alignX < -ALIGN_LIMIT || alignY > ALIGN_LIMIT || alignY < -ALIGN_LIMIT;
}

const int DIGIT_BLANK   = -1;
const int DIGIT_UNKNOWN = -2;

int maskToDigit(int mask) {
  switch (mask) {
    case 0x7E: return 0;
    case 0x30: return 1;
    case 0x6D: return 2;
    case 0x79: return 3;
    case 0x33: return 4;
    case 0x5B: return 5;
    case 0x5F: return 6;
    case 0x1F: return 6;   // 6 without the top bar
    case 0x72: return 7;   // 7 with the top left hook (what the UA-611 shows)
    case 0x70: return 7;
    case 0x7F: return 8;
    case 0x7B: return 9;
    case 0x73: return 9;   // 9 without the bottom bar
    case 0x00: return DIGIT_BLANK;
    default:   return DIGIT_UNKNOWN;
  }
}

// Returns the field's value, or -1 if it could not be read.
int readField(const uint8_t* img, const Field& f) {
  int value = 0;
  bool started = false;
  for (int i = 0; i < f.nDigits; i++) {
    int d = maskToDigit(segmentMask(img, f.digits[i], f.pct));
    if (d == DIGIT_UNKNOWN) return -1;
    if (d == DIGIT_BLANK) {
      if (started) return -1;   // blank in the middle of a number
      continue;                 // leading blank is fine
    }
    started = true;
    value = value * 10 + d;
  }
  return started ? value : -1;
}

// ---------------- Exposure ----------------

const int AEC_MIN  = 1;
const int AEC_MAX  = 1200;
const int GAIN_MAX = 30;

int lastBg  = -1;

// Median background level across the holes of every digit box.
int lcdBackground(const uint8_t* img) {
  int v[NUM_FIELDS * 3 * 2];
  int n = 0;
  for (int f = 0; f < NUM_FIELDS; f++) {
    for (int i = 0; i < fields[f].nDigits; i++) {
      for (int p = 0; p < 2; p++) {
        int s = holeLevel(img, fields[f].digits[i], p);
        int j = n++;
        while (j > 0 && v[j - 1] > s) { v[j] = v[j - 1]; j--; }
        v[j] = s;
      }
    }
  }
  return v[n / 2];
}

// Steps exposure towards TARGET_BG when the LCD background has drifted out of
// band. Returns true if it changed a setting, meaning this frame was too
// bright or too dark to trust and the next one should be better.
bool adjustExposure(const uint8_t* img) {
  lastBg = lcdBackground(img);
  if (!CALIBRATED || !AUTO_EXPOSURE) return false;
  if (lastBg >= TARGET_BG - BG_BAND && lastBg <= TARGET_BG + BG_BAND) return false;

  int aec = aecNow;
  int gain = gainNow;
  if (lastBg > TARGET_BG) {                       // too bright
    if (gain > AGC_GAIN) {
      gain = gain * 2 / 3;
      if (gain < AGC_GAIN) gain = AGC_GAIN;
    } else if (aec > AEC_MIN) {
      // A clipped frame hides how far over it is, so just halve.
      int want = (lastBg >= 245) ? aec / 2 : (int)((long)aec * TARGET_BG / lastBg);
      if (want > aec - 1) want = aec - 1;
      if (want < aec / 2) want = aec / 2;         // at most halve per step
      aec = want < AEC_MIN ? AEC_MIN : want;
    } else if (gain > 0) {
      gain--;
    }
  } else {                                        // too dark
    if (aec < AEC_MAX) {
      int want = (int)((long)aec * TARGET_BG / (lastBg < 1 ? 1 : lastBg));
      if (want < aec + 1) want = aec + 1;
      if (want > aec * 2) want = aec * 2;         // at most double per step
      aec = want > AEC_MAX ? AEC_MAX : want;
    } else if (gain < GAIN_MAX) {
      gain += (gain / 2 > 1) ? gain / 2 : 1;
      if (gain > GAIN_MAX) gain = GAIN_MAX;
    }
  }
  if (aec == aecNow && gain == gainNow) return false;   // at the limit, nothing left to try

  aecNow = aec;
  gainNow = gain;
  sensor_t* s = esp_camera_sensor_get();
  s->set_aec_value(s, aecNow);
  s->set_agc_gain(s, gainNow);
  return true;
}

// ---------------- Reading ----------------

const int READ_OK         = 0;  // all three fields decoded
const int READ_UNREADABLE = 1;  // exposure was fine but there were no valid digits (display off or mid-measurement)
const int READ_ADJUSTING  = 2;  // too bright or too dark, exposure has just been changed, look again shortly

int readOnce(int out[]) {
  camera_fb_t* fb = grabFrame(true);
  if (!fb) return READ_UNREADABLE;
  if (adjustExposure(fb->buf)) {
    esp_camera_fb_return(fb);
    return READ_ADJUSTING;
  }
  bool ok = true;
  frameMisaligned = !updateAlignment(fb->buf);
  if (frameMisaligned || alignmentLost()) {
    // Rows disagree, or moved too far to trust. If it is too far, start looking
    // from the calibrated place again.
    if (alignmentLost()) { alignX = 0; alignY = 0; }
    frameMisaligned = true;
    esp_camera_fb_return(fb);
    return READ_UNREADABLE;
  }
  for (int i = 0; i < NUM_FIELDS; i++) {
    out[i] = readField(fb->buf, fields[i]);
    if (out[i] < 0) ok = false;
  }
  esp_camera_fb_return(fb);
  return ok ? READ_OK : READ_UNREADABLE;
}

// Rejects the power-on segment test, mid-measurement frames and misreads.
// Field order is sys, dia, pul.
bool plausible(const int v[]) {
  if (v[0] < 60 || v[0] > 280) return false;
  if (v[1] < 30 || v[1] > 200) return false;
  if (v[2] < 30 || v[2] > 200) return false;
  return v[0] > v[1];
}

// Keeps reading until two consecutive frames agree.
bool readStable(int out[]) {
  int prev[NUM_FIELDS];
  bool havePrev = false;
  for (int attempt = 0; attempt < 12; attempt++) {
    int cur[NUM_FIELDS];
    if (readOnce(cur) == READ_OK && plausible(cur)) {
      if (havePrev && memcmp(cur, prev, sizeof(cur)) == 0) {
        memcpy(out, cur, sizeof(cur));
        return true;
      }
      memcpy(prev, cur, sizeof(cur));
      havePrev = true;
    } else {
      havePrev = false;
    }
    delay(100);
  }
  return false;
}

// ---------------- Auto-detect ----------------

int  candVals[NUM_FIELDS];
int  candCount    = 0;
int  invalidCount = 0;
bool reported     = false;

int  lastVals[NUM_FIELDS];
bool haveLast = false;
unsigned long lastAt = 0;
unsigned long lastPoll = 0;
bool retrySoon = false;   // exposure is still settling, poll again quickly
int  misalignedPolls = 0; // polls in a row where digits sat off their boxes
const int MISALIGNED_WARN  = 3;
const int MISALIGNED_RESET = 5;

// The latest result, waiting for A or B.
bool pendingReady = false;
int  pendingVals[NUM_FIELDS];

// Times (millis) the sleep rules work from.
unsigned long startPressAt = 0;   // last START press, or boot
unsigned long lastLitAt    = 0;   // last frame with anything on the display
unsigned long readingAt    = 0;   // when the pending result appeared
unsigned long lastWebAt    = 0;   // last web request (maintenance mode)
bool stayAwake   = false;         // set by an OTA upload, lasts until the next reboot
bool maintenance = false;         // WiFi, web page and OTA on, no sleep

void pollDisplay() {
  if (!CALIBRATED) return;   // the boxes do not match the display yet
  int cur[NUM_FIELDS];
  int status = readOnce(cur);
  retrySoon = (status == READ_ADJUSTING);
  // A frame skipped for exposure says nothing about the display, so it counts
  // neither towards a reading nor towards the display having cleared.
  if (retrySoon) return;
  if (litSegments >= 3) lastLitAt = millis();   // something is on the display
  if (frameMisaligned) {
    misalignedPolls++;
    if (misalignedPolls == MISALIGNED_WARN) {
      Serial.println("WARNING: digits are not where the boxes expect. The camera may have moved, recalibrate.");
    }
    if (misalignedPolls % MISALIGNED_RESET == 0) {
      // Lost track: look again from the calibrated place, in case it was put back.
      alignX = 0;
      alignY = 0;
    }
  } else {
    misalignedPolls = 0;
  }
  if (status == READ_OK && plausible(cur)) {
    invalidCount = 0;
    if (candCount > 0 && memcmp(cur, candVals, sizeof(cur)) == 0) {
      candCount++;
    } else {
      memcpy(candVals, cur, sizeof(cur));
      candCount = 1;
    }
    if (candCount >= STABLE_POLLS && !reported) {
      memcpy(lastVals, candVals, sizeof(candVals));
      haveLast = true;
      lastAt = millis();
      reported = true;
      memcpy(pendingVals, candVals, sizeof(candVals));
      pendingReady = true;
      readingAt = millis();
      Serial.printf("READING sys=%d dia=%d pul=%d\n", pendingVals[0], pendingVals[1], pendingVals[2]);
    }
  } else {
    candCount = 0;
    if (++invalidCount >= CLEAR_POLLS) reported = false;  // display cleared, ready for the next one
  }
}

// ---------------- Buttons, START and LED ----------------

// Presses are counted in interrupts so a quick tap is not missed while a
// frame is being captured.
volatile uint32_t pressesA = 0;
volatile uint32_t pressesB = 0;
volatile unsigned long edgeA = 0;
volatile unsigned long edgeB = 0;
volatile uint32_t startPresses = 0;
volatile unsigned long edgeS = 0;

int selectedUser = -1;   // 0 = button A, 1 = button B, -1 = nobody picked yet

bool isDown(int pin) {
  return pin >= 0 && digitalRead(pin) == LOW;
}

const char* userName(int u) {
  return (u == 0 || u == 1) ? USER_NAMES[u] : "?";
}

void IRAM_ATTR isrA() {
  unsigned long now = millis();
  if (now - edgeA > DEBOUNCE_MS && isDown(BTN_A)) pressesA++;
  edgeA = now;
}

void IRAM_ATTR isrB() {
  unsigned long now = millis();
  if (now - edgeB > DEBOUNCE_MS && isDown(BTN_B)) pressesB++;
  edgeB = now;
}

void IRAM_ATTR isrStart() {
  unsigned long now = millis();
  bool active = (digitalRead(WAKE_PIN) == HIGH) == WAKE_ACTIVE_HIGH;
  if (now - edgeS > DEBOUNCE_MS && active) startPresses++;
  edgeS = now;
}

bool wakeActive() {
  return WAKE_PIN >= 0 && (digitalRead(WAKE_PIN) == HIGH) == WAKE_ACTIVE_HIGH;
}

void ledSet(bool on) {
  if (STATUS_LED_PIN < 0) return;
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
}

void blink(int times, int ms) {
  for (int i = 0; i < times; i++) {
    ledSet(true);
    delay(ms);
    ledSet(false);
    delay(ms);
  }
}

void initInputs() {
  if (BTN_A >= 0) {
    pinMode(BTN_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_A), isrA, CHANGE);
  }
  if (BTN_B >= 0) {
    pinMode(BTN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_B), isrB, CHANGE);
  }
  if (WAKE_PIN >= 0) {
    pinMode(WAKE_PIN, WAKE_ACTIVE_HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WAKE_PIN), isrStart, CHANGE);
  }
}

// A START press either starts a measurement or switches the monitor off.
// Either way the display gets a fresh BLANK_SLEEP_MS to show something.
void onStartPressed() {
  startPressAt = millis();
  lastLitAt = millis();
}

void selectUser(int u) {
  selectedUser = u;
  Serial.printf("User: %s\n", userName(u));
  blink(u + 1, 120);
}

void startMaintenance(const char* why);

void checkInputs() {
  static uint32_t seenA = 0, seenB = 0, seenS = 0;
  static unsigned long bothSince = 0;

  uint32_t a = pressesA, b = pressesB, st = startPresses;
  if (a != seenA) { seenA = a; selectUser(0); }
  if (b != seenB) { seenB = b; selectUser(1); }
  if (st != seenS) {
    seenS = st;
    Serial.println("START pressed");
    onStartPressed();
  }

  // A and B held together: maintenance mode.
  bool both = isDown(BTN_A) && isDown(BTN_B);
  if (!both) {
    bothSince = 0;
  } else if (bothSince == 0) {
    bothSince = millis();
    if (bothSince == 0) bothSince = 1;
  } else if (!maintenance && millis() - bothSince >= MAINT_HOLD_MS) {
    startMaintenance("A and B held");
  }
}

// ---------------- Sending ----------------

// Readings waiting to be sent (Queued is declared near the top).
// Kept in RTC memory, so one that could not be sent is tried again with the
// next reading. Lost if power is removed.
const int QUEUE_MAX = 8;
RTC_DATA_ATTR Queued queue[QUEUE_MAX];
RTC_DATA_ATTR int queueLen = 0;

// Remembered between wakes so reconnecting is quicker: the access point it
// joined last time.
RTC_DATA_ATTR uint8_t  savedBssid[6];
RTC_DATA_ATTR int32_t  savedChannel  = 0;

void enqueue(int user, const int v[]) {
  if (queueLen == QUEUE_MAX) {   // full: drop the oldest
    memmove(queue, queue + 1, sizeof(Queued) * (QUEUE_MAX - 1));
    queueLen--;
  }
  Queued q;
  q.user = (uint8_t)user;
  q.sys  = (uint16_t)v[0];
  q.dia  = (uint16_t)v[1];
  q.pul  = (uint16_t)v[2];
  queue[queueLen++] = q;
}

void dequeueFirst() {
  memmove(queue, queue + 1, sizeof(Queued) * (queueLen - 1));
  queueLen--;
}

bool waitForWifi(unsigned long ms) {
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < ms) delay(50);
  return WiFi.status() == WL_CONNECTED;
}

bool wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  if (USE_STATIC_IP) WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER);
  bool ok = false;
  if (savedChannel > 0) {
    // Straight to the access point used last time, skipping the channel scan.
    WiFi.begin(WIFI_SSID, WIFI_PASS, savedChannel, savedBssid);
    ok = waitForWifi(WIFI_TIMEOUT_MS / 2);
    if (!ok) {
      WiFi.disconnect();
      savedChannel = 0;
    }
  }
  if (!ok) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    ok = waitForWifi(WIFI_TIMEOUT_MS);
  }
  if (ok) {
    savedChannel = WiFi.channel();
    memcpy(savedBssid, WiFi.BSSID(), 6);
  }
  return ok;
}

void wifiOff() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// Sends one reading. Returns the HTTP status, or a negative number if the
// server could not be reached. Change this to send readings somewhere else.
int sendReading(const Queued& q) {
  char body[128];
  snprintf(body, sizeof(body), "{\"user\":\"%s\",\"systolic\":%d,\"diastolic\":%d,\"pulse\":%d}",
           userName(q.user), q.sys, q.dia, q.pul);
  WiFiClient plain;
  WiFiClientSecure secure;
  secure.setInsecure();   // https is encrypted but the server's certificate is not checked
  bool https = strncmp(SEND_URL, "https:", 6) == 0;
  HTTPClient http;
  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!(https ? http.begin(secure, SEND_URL) : http.begin(plain, SEND_URL))) return -1;
  http.addHeader("Content-Type", "application/json");
  if (strlen(SEND_AUTH) > 0) http.addHeader("Authorization", SEND_AUTH);
  int code = http.POST(body);
  http.end();
  return code;
}

// Sends everything queued. Returns true if the queue is now empty.
bool sendQueue() {
  if (queueLen == 0) return true;
  if (!wifiConnect()) {
    Serial.println("Send: WiFi did not connect, kept for next time");
    return false;
  }
  while (queueLen > 0) {
    int code = sendReading(queue[0]);
    if (code < 0 || code >= 500) {
      Serial.printf("Send: failed (%d), %d kept for next time\n", code, queueLen);
      return false;
    }
    // 2xx means stored. Anything else (400: values rejected) will not get
    // better by trying again, so it is dropped either way.
    Serial.printf("Send: %s %d/%d/%d -> HTTP %d\n", userName(queue[0].user),
                  queue[0].sys, queue[0].dia, queue[0].pul, code);
    dequeueFirst();
  }
  return true;
}

// ---------------- Sleep ----------------

void goToSleep(const char* why) {
  Serial.print("Going to sleep: ");
  Serial.println(why);
  // Waking is level triggered: if START were still held, it would wake straight away.
  unsigned long t0 = millis();
  while (wakeActive() && millis() - t0 < 5000) delay(10);

  wifiOff();
  cameraOff();
  // Camera powered down and both LEDs off, held that way through sleep.
  rtc_gpio_hold_en((gpio_num_t)CAM_PWDN_PIN);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);
  rtc_gpio_hold_en((gpio_num_t)FLASH_PIN);
  if (STATUS_LED_PIN >= 0) {
    ledSet(false);
    rtc_gpio_hold_en((gpio_num_t)STATUS_LED_PIN);
  }

  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_PIN, WAKE_ACTIVE_HIGH ? 1 : 0);
  if (WAKE_ACTIVE_HIGH) {
    rtc_gpio_pullup_dis((gpio_num_t)WAKE_PIN);
    rtc_gpio_pulldown_en((gpio_num_t)WAKE_PIN);
  } else {
    rtc_gpio_pulldown_dis((gpio_num_t)WAKE_PIN);
    rtc_gpio_pullup_en((gpio_num_t)WAKE_PIN);
  }
  Serial.flush();
  esp_deep_sleep_start();
}

// Undoes what goToSleep() held. Call first thing in setup(), before the
// camera starts. Returns true if this boot was START waking it.
bool wakeFromSleep() {
  rtc_gpio_hold_dis((gpio_num_t)CAM_PWDN_PIN);
  rtc_gpio_hold_dis((gpio_num_t)FLASH_PIN);
  if (STATUS_LED_PIN >= 0) rtc_gpio_hold_dis((gpio_num_t)STATUS_LED_PIN);
  bool byStart = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
  if (byStart) rtc_gpio_deinit((gpio_num_t)WAKE_PIN);
  return byStart;
}

// Which sleep rule is due now, or nullptr to stay awake.
const char* sleepDue() {
  if (!sleepMode() || maintenance || stayAwake) return nullptr;
  unsigned long now = millis();
  if (pendingReady) {
    // A result is waiting for A or B, even if the monitor has been switched off.
    if (now - readingAt >= USER_WAIT_MS) return "no user picked, reading discarded";
  } else if (now - lastLitAt >= BLANK_SLEEP_MS) {
    return "display blank";
  }
  if (now - startPressAt >= AWAKE_MAX_MS) return "awake too long";
  return nullptr;
}

// ---------------- Web handlers ----------------

// The picture reloads shortly after the previous frame has arrived.
// The status lines poll /state twice a second.
const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BP reader</title>
<body style="margin:0;background:#111;color:#ddd;font:15px monospace;text-align:center">
<img id="v" style="width:640px;max-width:100%;image-rendering:pixelated">
<p><label><input type="checkbox" id="bx"> show boxes</label>
&nbsp; <a href="/img?raw=1" download="frame.bmp" style="color:#8cf">save frame for the mapper</a>
&nbsp; <a href="/debug" style="color:#8cf">debug</a>
&nbsp; <a href="/read" style="color:#8cf">read now</a></p>
<pre id="s" style="margin:8px;font:15px monospace">connecting...</pre>
<script>
const v = document.getElementById('v');
const s = document.getElementById('s');
const bx = document.getElementById('bx');
const next = () => { v.src = '/img?t=' + Date.now() + (bx.checked ? '&boxes=1' : ''); };
v.onload = () => setTimeout(next, 150);
v.onerror = () => setTimeout(next, 1000);
next();

const btn = (name, down, count) => 'Button ' + name + ': ' + (down ? 'DOWN' : 'up  ') + ' (' + count + ' presses)';
const mins = (s) => Math.floor(s / 60) + 'm ' + (s % 60) + 's';
const poll = () => fetch('/state')
  .then(r => r.json())
  .then(d => {
    s.textContent = [
      !d.calibrated ? 'NOT CALIBRATED: run the digit mapper (step 2)'
        : d.misaligned ? 'CHECK CAMERA: digits are off their boxes, it may have moved' : 'Calibrated',
      d.have_reading ? 'Last reading: ' + d.sys + ' / ' + d.dia + ' / ' + d.pul + ' (' + d.age_s + 's ago)' : 'Last reading: none yet',
      'User: ' + (d.user || 'none picked') + ', readings waiting to send: ' + d.queued,
      btn('A', d.a, d.a_presses),
      btn('B', d.b, d.b_presses),
      'START: ' + (d.wake ? 'PRESSED' : 'idle') + ' (pin is ' + (d.wake_level ? 'high' : 'low') + ')',
      d.maint_left_s < 0 ? 'Web page: always on'
        : 'Web page: ends ' + mins(d.maint_left_s) + ' after you close this page'
    ].join('\n');
  })
  .catch(() => { s.textContent = 'no response'; })
  .finally(() => setTimeout(poll, 500));
poll();
</script>
)HTML";

void handleRoot() {
  lastWebAt = millis();
  server.send_P(200, "text/html", PAGE);
}

void setPixel(uint8_t* img, int x, int y, uint8_t v) {
  if (x < 0 || y < 0 || x >= IMG_W || y >= IMG_H) return;
  img[y * IMG_W + x] = v;
}

void drawOverlay(uint8_t* img) {
  uint8_t boxCol = DARK_ON_LIGHT ? 0 : 255;
  for (int f = 0; f < NUM_FIELDS; f++) {
    for (int i = 0; i < fields[f].nDigits; i++) {
      const Box& b = fields[f].digits[i];
      int bx = b.x + alignX, by = b.y + alignY;
      for (int x = bx; x < bx + b.w; x++) {
        setPixel(img, x, by, boxCol);
        setPixel(img, x, by + b.h - 1, boxCol);
      }
      for (int y = by; y < by + b.h; y++) {
        setPixel(img, bx, y, boxCol);
        setPixel(img, bx + b.w - 1, y, boxCol);
      }
      // Segment sample points in white, background points in mid grey.
      for (int s = 0; s < 7; s++) {
        setPixel(img, ptX(b, SEG_PTS[s][0], SEG_PTS[s][1]), ptY(b, SEG_PTS[s][1]), 255);
      }
      for (int s = 0; s < 2; s++) {
        for (int k = 0; k < 3; k++) {
          setPixel(img, ptX(b, HOLE_X[k], HOLE_Y[s]), ptY(b, HOLE_Y[s]), 128);
        }
      }
    }
  }
}

void handleImg() {
  lastWebAt = millis();
  camera_fb_t* fb = grabFrame(!server.hasArg("raw"));
  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }
  if (server.hasArg("boxes")) drawOverlay(fb->buf);
  uint8_t* bmp = nullptr;
  size_t bmpLen = 0;
  bool ok = frame2bmp(fb, &bmp, &bmpLen);
  esp_camera_fb_return(fb);
  if (!ok) { server.send(500, "text/plain", "bmp conversion failed"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/bmp", (const char*)bmp, bmpLen);
  free(bmp);
}

void handleDebug() {
  lastWebAt = millis();
  camera_fb_t* fb = grabFrame(true);
  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }
  if (CALIBRATED) updateAlignment(fb->buf);
  int lcdBg = lcdBackground(fb->buf);
  String out = "";
  if (!CALIBRATED) {
    out += "NOT CALIBRATED: the boxes may not match the camera position, so the numbers below may mean nothing\n";
    out += "exposure: sensor automatic\n";
  } else {
    out += "exposure " + String(aecNow) + ", gain " + String(gainNow);
    out += AUTO_EXPOSURE ? " (auto)\n" : " (fixed)\n";
  }
  out += "LCD background " + String(lcdBg) + ", target " + String(TARGET_BG - BG_BAND) + " to " + String(TARGET_BG + BG_BAND) + "\n";
  out += "alignment: digits are " + String(alignX) + "px right, " + String(alignY) + "px down of where they were calibrated\n";
  out += "segment order: abcdefg. 1 = lit, 0 = unlit, ? = unsure (too close to the threshold)\n";
  out += "contrast is how much darker each segment is than its background, in %\n\n";
  for (int f = 0; f < NUM_FIELDS; f++) {
    int pct = fields[f].pct;
    out += String(fields[f].name) + ": lit above " + String(pct + GUARD_PCT) + "%, unlit below " + String(pct - GUARD_PCT) + "%\n";
    for (int i = 0; i < fields[f].nDigits; i++) {
      int diff[7], ref[7];
      segmentDiffs(fb->buf, fields[f].digits[i], diff, ref);
      int d = maskToDigit(segmentMask(fb->buf, fields[f].digits[i], pct));
      out += "  digit " + String(i) + "  ";
      for (int s = 0; s < 7; s++) {
        int c = classify(diff[s], ref[s], pct);
        out += (c == SEG_ON) ? '1' : (c == SEG_OFF) ? '0' : '?';
      }
      out += "  -> ";
      if (d == DIGIT_BLANK) out += "blank";
      else if (d == DIGIT_UNKNOWN) out += "?";
      else out += String(d);
      out += "   background " + String(ref[6]) + ", contrast";
      for (int s = 0; s < 7; s++) out += " " + String(ref[s] > 0 ? diff[s] * 100 / ref[s] : 0);
      out += "\n";
    }
    out += "  value: " + String(readField(fb->buf, fields[f])) + "\n\n";
  }
  esp_camera_fb_return(fb);
  server.send(200, "text/plain", out);
}

String valsToJson(const int v[]) {
  String json = "";
  for (int i = 0; i < NUM_FIELDS; i++) {
    json += ",\"" + String(fields[i].name) + "\":" + String(v[i]);
  }
  return json;
}

void handleRead() {
  lastWebAt = millis();
  if (!CALIBRATED) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"not calibrated\"}");
    return;
  }
  int vals[NUM_FIELDS];
  bool ok = readStable(vals);
  String json = "{\"ok\":";
  json += ok ? "true" : "false";
  if (ok) json += valsToJson(vals);
  json += "}";
  server.send(ok ? 200 : 503, "application/json", json);
}

void handleLast() {
  lastWebAt = millis();
  String json = "{\"ok\":";
  json += haveLast ? "true" : "false";
  if (haveLast) {
    json += valsToJson(lastVals);
    json += ",\"age_s\":" + String((millis() - lastAt) / 1000);
  }
  json += "}";
  server.send(200, "application/json", json);
}

void handleState() {
  lastWebAt = millis();
  String json = "{\"calibrated\":";
  json += CALIBRATED ? "true" : "false";
  json += ",\"have_reading\":";
  json += haveLast ? "true" : "false";
  if (haveLast) {
    json += valsToJson(lastVals);
    json += ",\"age_s\":" + String((millis() - lastAt) / 1000);
  }
  json += ",\"user\":";
  json += selectedUser < 0 ? String("null") : "\"" + String(userName(selectedUser)) + "\"";
  json += ",\"queued\":" + String(queueLen);
  json += ",\"a\":" + String(isDown(BTN_A) ? 1 : 0);
  json += ",\"a_presses\":" + String((unsigned long)pressesA);
  json += ",\"b\":" + String(isDown(BTN_B) ? 1 : 0);
  json += ",\"b_presses\":" + String((unsigned long)pressesB);
  json += ",\"wake\":" + String(wakeActive() ? 1 : 0);
  json += ",\"wake_level\":" + String(WAKE_PIN >= 0 && digitalRead(WAKE_PIN) == HIGH ? 1 : 0);
  json += ",\"misaligned\":";
  json += (misalignedPolls >= MISALIGNED_WARN) ? "true" : "false";
  json += ",\"align_x\":" + String(alignX) + ",\"align_y\":" + String(alignY);
  json += ",\"maint_left_s\":" + String(sleepMode() ? (long)(MAINT_IDLE_MS / 1000) : -1L);
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// ---------------- OTA ----------------

void initOTA() {
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA
    .onStart([]() {
      stayAwake = true;
      Serial.println("OTA start");
    })
    .onEnd([]() {
      Serial.println("OTA done, rebooting");
    })
    .onProgress([](unsigned int done, unsigned int total) {
      static uint32_t last = 0;
      if (total > 0 && millis() - last > 500) {
        last = millis();
        Serial.printf("OTA %u%%\n", (unsigned int)(done * 100ULL / total));
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("OTA error %u\n", (unsigned int)error);
    });
  ArduinoOTA.begin();
}

// ---------------- Maintenance mode and main flow ----------------

bool servicesUp = false;   // web server and OTA running
unsigned long servicesTriedAt = 0;

void startServices() {
  servicesTriedAt = millis();
  if (!wifiConnect()) {
    Serial.println("WiFi did not connect: no web page or OTA yet");
    return;
  }
  if (!servicesUp) {
    server.on("/", handleRoot);
    server.on("/img", handleImg);
    server.on("/debug", handleDebug);
    server.on("/read", handleRead);
    server.on("/last", handleLast);
    server.on("/state", handleState);
    server.begin();
    initOTA();
    servicesUp = true;
  }
  Serial.printf("Web page and OTA: http://%s  (or http://%s.local)\n",
                WiFi.localIP().toString().c_str(), HOSTNAME);
}

void startMaintenance(const char* why) {
  if (maintenance) return;
  Serial.printf("Maintenance mode (%s)\n", why);
  maintenance = true;
  selectedUser = hasButtons() ? -1 : 0;   // holding A and B is not picking a user
  lastWebAt = millis();
  blink(5, 60);
  startServices();
  if (!servicesUp && sleepMode()) maintenance = false;   // nothing to stay awake for
}

void endMaintenance() {
  maintenance = false;
  Serial.println("Maintenance mode over");
  if (servicesUp) {
    ArduinoOTA.end();
    server.stop();
    servicesUp = false;
  }
  wifiOff();
}

// Sends a result once it has a user, then sleeps (outside maintenance mode).
void handleResult() {
  if (!pendingReady) return;
  if (selectedUser < 0) {
    if (!sleepMode() && millis() - readingAt >= USER_WAIT_MS) {
      pendingReady = false;
      Serial.println("No user picked, reading discarded");
    }
    return;
  }
  enqueue(selectedUser, pendingVals);
  pendingReady = false;
  if (hasButtons()) selectedUser = -1;
  if (!maintenance) cameraOff();   // done reading, and WiFi plus camera can brown the board out
  bool ok = sendQueue();
  if (ok) blink(1, 800);
  else    blink(3, 120);
  if (!maintenance) goToSleep(ok ? "reading sent" : "send failed, queued for next time");
}

// ---------------- Setup ----------------

void setup() {
  bool wokeByStart = wakeFromSleep();
  Serial.begin(115200);
  Serial.println(wokeByStart ? "Woken by START" : "Power on / reset");
  if (STATUS_LED_PIN >= 0) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    ledSet(false);
  }
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  if (!initCamera()) {
    Serial.println("Camera init failed");
    if (wokeByStart) goToSleep("camera init failed");
    while (true) delay(1000);
  }
  initInputs();   // after the camera, which sets up the interrupt service first

  // The press that woke it (or power-on) starts the sleep timers.
  onStartPressed();
  if (queueLen > 0) Serial.printf("%d reading(s) queued from before\n", queueLen);

  if (!hasButtons()) selectedUser = 0;
  if (!CALIBRATED) {
    Serial.println("Not calibrated: reading and sleep are off until the digit map is filled in");
    startMaintenance("not calibrated");
  } else if (!sleepMode()) {
    startMaintenance("always on");
  } else if (!wokeByStart) {
    startMaintenance("power-on or reset");
  }
}

void loop() {
  if (servicesUp) {
    ArduinoOTA.handle();
    server.handleClient();
  }
  checkInputs();
  unsigned long wait = retrySoon ? RETRY_MS : POLL_MS;
  if (millis() - lastPoll >= wait) {
    lastPoll = millis();
    pollDisplay();
  }
  handleResult();
  if (maintenance && !servicesUp && millis() - servicesTriedAt >= 30000) startServices();
  if (maintenance && sleepMode() && !stayAwake && millis() - lastWebAt >= MAINT_IDLE_MS) {
    endMaintenance();
  }
  const char* why = sleepDue();
  if (why) goToSleep(why);
}
