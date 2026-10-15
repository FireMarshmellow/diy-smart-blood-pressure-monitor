// Step 1: live camera preview for the AI-Thinker ESP32-CAM.
//
// Upload this, open the Serial Monitor at 115200 and go to the address it
// prints (or http://bp-camera.local). Position the camera so the whole
// display fills as much of the picture as possible, take a reading on the
// monitor, and press "Save frame" while the result is showing. That frame
// goes into step 2.
//
// Board: "AI Thinker ESP32-CAM", PSRAM enabled.

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "img_converters.h"

// Fill these in, or put them in a secrets.h next to this file (git-ignored).
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID "your-wifi-name"
#define WIFI_PASS "your-wifi-password"
#endif

const char* HOSTNAME = "bp-camera";
const int   FLASH_PIN = 4;

WebServer server(80);

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = 5;   c.pin_d1 = 18;  c.pin_d2 = 19;  c.pin_d3 = 21;
  c.pin_d4 = 36;  c.pin_d5 = 39;  c.pin_d6 = 34;  c.pin_d7 = 35;
  c.pin_xclk = 0; c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sccb_sda = 26; c.pin_sccb_scl = 27;
  c.pin_pwdn = 32; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;   // the reader works on greyscale 320x240 frames
  c.frame_size   = FRAMESIZE_QVGA;
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  return esp_camera_init(&c) == ESP_OK;
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BP camera preview</title>
<body style="margin:0;background:#111;color:#ddd;font:15px sans-serif;text-align:center">
<img id="v" style="width:640px;max-width:100%;image-rendering:pixelated;margin-top:12px">
<p>
  <a id="save" href="/img" download="frame.bmp" style="color:#8cf">Save frame</a>
  &nbsp; <label><input type="checkbox" id="rot"> turn view 180&deg;</label>
  &nbsp; <label><input type="checkbox" id="fl"> flash</label>
</p>
<p style="color:#888;max-width:600px;margin:auto">Save the frame while a result is on the display.
The saved file is always the camera's raw frame. If it is upside down, tick
"Rotate 180" in the digit mapper.</p>
<script>
const v = document.getElementById('v');
const rot = document.getElementById('rot');
const fl = document.getElementById('fl');
const next = () => { v.src = '/img?t=' + Date.now(); };
v.onload = () => setTimeout(next, 100);
v.onerror = () => setTimeout(next, 1000);
rot.onchange = () => { v.style.transform = rot.checked ? 'rotate(180deg)' : ''; };
fl.onchange = () => fetch('/flash?on=' + (fl.checked ? 1 : 0));
next();
</script>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleImg() {
  camera_fb_t* fb = esp_camera_fb_get();   // first one may be stale
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }
  uint8_t* bmp = nullptr;
  size_t len = 0;
  bool ok = frame2bmp(fb, &bmp, &len);
  esp_camera_fb_return(fb);
  if (!ok) { server.send(500, "text/plain", "bmp conversion failed"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/bmp", (const char*)bmp, len);
  free(bmp);
}

void handleFlash() {
  digitalWrite(FLASH_PIN, server.arg("on") == "1" ? HIGH : LOW);
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  if (!initCamera()) {
    Serial.println("Camera init failed. Check the ribbon cable and that PSRAM is enabled.");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  MDNS.begin(HOSTNAME);

  server.on("/", handleRoot);
  server.on("/img", handleImg);
  server.on("/flash", handleFlash);
  server.begin();

  Serial.printf("Preview ready: http://%s  (or http://%s.local)\n",
                WiFi.localIP().toString().c_str(), HOSTNAME);
}

void loop() {
  server.handleClient();
}
