/*
 * ==============================================================================
 * AI-Thinker ESP32-CAM — High-Performance MJPEG Stream Server
 * For AI Smart Waste Management Robot (YOLOv8 Real-Time Detection)
 * ==============================================================================
 * 
 * Features:
 * - High-speed MJPEG video streaming on Port 81 (/stream) & Port 80 (/stream)
 * - Snapshot capture on /capture
 * - Flash LED toggle on /flash?state=1 or /flash?state=0
 * - Cold-boot brownout protection (auto-starts when powered on, no reset needed!)
 * - Automatic Wi-Fi reconnection in background
 * 
 * Arduino IDE Settings:
 *   Board:       "AI Thinker ESP32-CAM"
 *   CPU Freq:    240MHz (WiFi/BT)
 *   Flash Freq:  80MHz
 *   Flash Mode:  QIO
 *   Partition:   Huge APP (3MB No OTA / 1MB SPIFFS)
 *   PSRAM:       "Enabled" (CRITICAL for high FPS!)
 * ==============================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"             // Power / brownout control
#include "soc/rtc_cntl_reg.h"    // Power / brownout control
#include "esp_http_server.h"
#include <lwip/sockets.h>

// ------------------------------------------------------------------------------
// 1. PERMANENT FIXED STATIC IP CONFIGURATION
// ------------------------------------------------------------------------------
// Fixed Static IP on your Laptop Mobile Hotspot ("AADHIDHYA"):
const char *ssid         = "AADHIDHYA";
const char *password     = "12345678";
const IPAddress static_IP(192, 168, 137, 200);   // <-- FIXED CAMERA IP (NEVER CHANGES)
const IPAddress gateway  (192, 168, 137, 1);     // Laptop Hotspot Gateway
const IPAddress subnet   (255, 255, 255, 0);
const IPAddress dns      (192, 168, 137, 1);

// Direct Standalone Access Point (Fallback):
const char *ap_ssid      = "SmartWasteCam";
const char *ap_password  = "12345678";
const IPAddress ap_IP    (192, 168, 4, 1);       // <-- DIRECT AP FIXED IP
const IPAddress ap_gateway(192, 168, 4, 1);
const IPAddress ap_subnet(255, 255, 255, 0);

// ------------------------------------------------------------------------------
// 2. AI-THINKER ESP32-CAM PIN DEFINITIONS
// ------------------------------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN      4   // Onboard high-power white LED

// Stream boundary header
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ------------------------------------------------------------------------------
// 3. MJPEG STREAM HANDLER (Port 81 /stream)
// ------------------------------------------------------------------------------
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  // Optimize TCP socket: Disable Nagle algorithm & enforce 2s send timeout
  int sock = httpd_req_to_sockfd(req);
  if (sock >= 0) {
    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[CAM ERROR] Camera capture failed!");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.println("[CAM ERROR] JPEG conversion failed!");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }

    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) {
      break;
    }
  }
  return res;
}

// ------------------------------------------------------------------------------
// 4. STILL CAPTURE HANDLER (/capture)
// ------------------------------------------------------------------------------
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ------------------------------------------------------------------------------
// 5. FLASH LED HANDLER (/flash?state=1 or /flash?state=0)
// ------------------------------------------------------------------------------
static esp_err_t flash_handler(httpd_req_t *req) {
  char buf[32];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char val[8];
    if (httpd_query_key_value(buf, "state", val, sizeof(val)) == ESP_OK) {
      int state = atoi(val);
      digitalWrite(FLASH_LED_PIN, state ? HIGH : LOW);
      httpd_resp_send(req, state ? "FLASH ON" : "FLASH OFF", HTTPD_RESP_USE_STRLEN);
      return ESP_OK;
    }
  }
  httpd_resp_send(req, "Use /flash?state=1 or state=0", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// ------------------------------------------------------------------------------
// 6. START HTTP SERVERS (Port 80 & Port 81)
// ------------------------------------------------------------------------------
void startCameraServer() {
  if (stream_httpd != NULL || camera_httpd != NULL) {
    return; // Already running, prevent duplicate listen on ports
  }

  // Stream Server on Port 81 (Matches Mission Control: http://<IP>:81/stream)
  httpd_config_t config_stream = HTTPD_DEFAULT_CONFIG();
  config_stream.server_port = 81;
  config_stream.ctrl_port = 32769;

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config_stream) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &stream_root_uri);
    Serial.println("✅ Camera MJPEG Stream ready on Port 81: http://<CAM_IP>:81/stream");
  }

  // Web Control Server on Port 80 (/capture, /flash, and /stream fallback)
  httpd_config_t config_cam = HTTPD_DEFAULT_CONFIG();
  config_cam.server_port = 80;

  httpd_uri_t capture_uri = {
    .uri       = "/capture",
    .method    = HTTP_GET,
    .handler   = capture_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t flash_uri = {
    .uri       = "/flash",
    .method    = HTTP_GET,
    .handler   = flash_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream80_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&camera_httpd, &config_cam) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &flash_uri);
    httpd_register_uri_handler(camera_httpd, &stream80_uri);
    Serial.println("✅ Web Control Server ready on Port 80: http://<CAM_IP>/stream");
  }
}

// ------------------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------------------
void setup() {
  // 1. Disable brownout detector immediately so cold-boot startup spikes don't lock chip!
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1200); // Allow power supply to stabilize completely

  Serial.println("\n========================================================");
  Serial.println(" AI Smart Waste Robot — ESP32-CAM Video Streamer");
  Serial.println("========================================================");

  // Setup Flash LED
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // 2. Configure OV2640 Camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST; // CRITICAL: Always streams freshest frame (eliminates lag!)
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  // Ultra-Low Latency settings for YOLOv8 (25-30 FPS, Zero-Lag Wi-Fi)
  if (psramFound()) {
    Serial.println("✅ PSRAM Detected! Enabling High-Speed Double-Buffering.");
    config.frame_size   = FRAMESIZE_HVGA; // 480x320 (Optimal balance: high detail + ultra low latency)
    config.jpeg_quality = 21;             // Cuts payload to ~8KB -> 30 FPS zero-lag stream!
    config.fb_count     = 2;              // Double buffering
  } else {
    Serial.println("⚠️ Warning: PSRAM not detected. Using CIF 400x296.");
    config.frame_size   = FRAMESIZE_CIF;  // 400x296
    config.jpeg_quality = 24;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  // Camera Init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed with error 0x%x\n", err);
    Serial.println("   Please check camera ribbon cable connection and press Reset.");
    return;
  }

  // Optimize camera image sensors for waste detection (sharp contrast & color)
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 1);     // -2 to 2
    s->set_contrast(s, 1);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - 0: Auto
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  }

  // 3. Connect to Laptop Hotspot with Fixed Static IP (Zero Latency Mode)
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);               // CRITICAL: Disable modem sleep to eliminate latency & freezing
  esp_wifi_set_ps(WIFI_PS_NONE);      // CRITICAL: Continuous full-power RF transmission
  WiFi.softAPConfig(ap_IP, ap_gateway, ap_subnet);
  WiFi.softAP(ap_ssid, ap_password);

  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  WiFi.disconnect(false);
  WiFi.config(static_IP, gateway, subnet, dns);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(ssid, password);
  delay(200);

  Serial.println("\n========================================================");
  Serial.println("🎉 ESP32-CAM PERMANENT FIXED STATIC IP ACTIVE!");
  Serial.printf ("   Connecting to Hotspot : %s\n", ssid);
  Serial.println("   ----------------------------------------------------");
  Serial.println("   ★ Fixed Hotspot Stream : http://192.168.137.200:81/stream");
  Serial.println("   ★ Fixed Direct AP Stream: http://192.168.4.1:81/stream");
  Serial.println("   (IP Address is LOCKED and will NEVER change on reboot!)");
  Serial.println("========================================================");

  // Immediately start high-performance streaming server!
  startCameraServer();
}

// ------------------------------------------------------------------------------
// MAIN LOOP
// ------------------------------------------------------------------------------
void loop() {
  if (stream_httpd == NULL && camera_httpd == NULL) {
    startCameraServer();
  }

  // Ensure hotspot connection with fixed static IP remains active
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 4000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.config(static_IP, gateway, subnet, dns);
      WiFi.begin(ssid, password);
    }
  }

  delay(250);
}
