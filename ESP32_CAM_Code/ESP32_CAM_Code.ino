#include "esp_camera.h"
#include "soc/rtc_cntl_reg.h" // Used to disable brownout detector
#include "soc/soc.h"          // Used to disable brownout detector
#include <WebServer.h>
#include <WiFi.h>


// ================= WIFI CONFIG =================
const char *ssid = "Creoleap-2.4_3560";
const char *password = "Creo@0325";

// ================= SERVO CONFIG =================
#define SERVO_PLASTIC 12
#define SERVO_MEDICAL 13
#define SERVO_GENERAL 14

#define SERVO_FREQ 50 // 50Hz for servo
#define SERVO_RES 16  // 16-bit resolution for smoother movement

#define OPEN_POS 100 // Target angle in degrees
#define CLOSE_POS 15 // Target angle in degrees

// ================= BUZZER CONFIG =================
#define BUZZER_PIN 15
#define BUZZER_FREQ 2000
#define BUZZER_RES 8

// ================= CAMERA PINOUT (AI THINKER) ==
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

WebServer server(80);

// ================= SERVO UTILS =================
uint32_t angleToDuty(int angle) {
  // map(angle, 0, 180, 0.5ms duty, 2.5ms duty)
  // 0.5ms / 20ms * 65536 = 1638
  // 2.5ms / 20ms * 65536 = 8192
  return map(angle, 0, 180, 1638, 8192);
}

// ================= BUZZER ======================
void beepBuzzer(int durationMs) {
  ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);
  delay(durationMs);
  ledcWriteTone(BUZZER_PIN, 0);
}

// ================= SERVO MOVE ==================
void moveServo(uint8_t pin) {
  beepBuzzer(150);

  // Open lid smoothly
  for (int pos = CLOSE_POS; pos <= OPEN_POS; pos += 5) {
    ledcWrite(pin, angleToDuty(pos));
    delay(15);
  }

  delay(2000); // Hold open

  // Close lid smoothly
  for (int pos = OPEN_POS; pos >= CLOSE_POS; pos -= 5) {
    ledcWrite(pin, angleToDuty(pos));
    delay(15);
  }
}

// ================= CAMERA HANDLERS =============
void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ================= CONTROL =====================
void handleControl() {
  if (!server.hasArg("bin")) {
    server.send(400, "text/plain", "Missing bin argument");
    return;
  }

  String binType = server.arg("bin");
  Serial.print("Opening Bin: ");
  Serial.println(binType);

  if (binType == "plastic") {
    moveServo(SERVO_PLASTIC);
  } else if (binType == "medical") {
    moveServo(SERVO_MEDICAL);
  } else if (binType == "general") {
    moveServo(SERVO_GENERAL);
  }

  server.send(200, "text/plain", "OK");
}

// ================= SETUP =======================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

  Serial.begin(115200);

  // -------- Servo PWM Setup --------
  ledcAttach(SERVO_PLASTIC, SERVO_FREQ, SERVO_RES);
  ledcAttach(SERVO_MEDICAL, SERVO_FREQ, SERVO_RES);
  ledcAttach(SERVO_GENERAL, SERVO_FREQ, SERVO_RES);

  // Initialize servos to closed position
  ledcWrite(SERVO_PLASTIC, angleToDuty(CLOSE_POS));
  ledcWrite(SERVO_MEDICAL, angleToDuty(CLOSE_POS));
  ledcWrite(SERVO_GENERAL, angleToDuty(CLOSE_POS));

  // -------- Buzzer Setup --------
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES);

  // -------- Camera Setup --------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Optimized for speed
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12; // Lower number = higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 14;
    config.fb_count = 1;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera Init Failed");
    return;
  }

  // -------- WiFi --------
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Stream URL: http://" + WiFi.localIP().toString() +
                 "/capture");

  server.on("/capture", handleCapture);
  server.on("/control", handleControl);
  server.begin();
}

void loop() { server.handleClient(); }
