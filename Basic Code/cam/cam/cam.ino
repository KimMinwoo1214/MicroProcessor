/********************************************************************
 * ESP32-CAM : Red Area Detection
 *   – 빨강 비율이 0.27 이상인 프레임이 5 연속 → 두 핀 HIGH
 *   – 빨강 비율이 0.23 이하인 프레임이 5 연속 → 두 핀 LOW
 *   – ON 상태 : GPIO12, GPIO4 모두 HIGH (100 %)
 ********************************************************************/
#include <Arduino.h>
#include "esp_camera.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

/* ---------- 사용자 설정 ---------- */
#define PIN_LED1      12          // LED 1 (100 %)
#define PIN_LED2      4           // LED 2 (100 %)
#define FRAME_SIZE    FRAMESIZE_QQVGA   // 160×120
#define TH_ON_RATIO   0.23        // 켤 때 기준
#define TH_OFF_RATIO  0.18        // 끌 때 기준
#define TH_FRAMES     5           // 연속 프레임 수
/* ---------------------------------- */

/* 빨강 판별 함수 (RGB565 → 8-bit RGB) */
inline bool isRed(uint16_t rgb565)
{
  uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((rgb565 >>  5) & 0x3F) * 255 / 63;
  uint8_t b = ( rgb565        & 0x1F) * 255 / 31;
  return (r > 120) && (r > g + 30) && (r > b + 30);
}

void setup()
{
  Serial.begin(115200);

  /* 출력 핀 초기화 */
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);

  /* 카메라 설정 */
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;  cfg.pin_d2 = Y4_GPIO_NUM;
  cfg.pin_d3 = Y5_GPIO_NUM;  cfg.pin_d4 = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM; cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM; cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_RGB565;   // RAW
  cfg.frame_size   = FRAME_SIZE;
  cfg.jpeg_quality = 12;
  cfg.fb_count     = 1;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true) delay(1000);
  }
  Serial.println("Start red-area detection");
}

void loop()
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  /* 빨강 픽셀 계산 */
  uint16_t *pix = (uint16_t *)fb->buf;
  size_t tot = fb->len >> 1, red = 0;
  for (size_t i = 0; i < tot; ++i)
    if (isRed(pix[i])) ++red;
  esp_camera_fb_return(fb);

  float ratio = (float)red / tot;

  /* 히스테리시스 + 디바운스 */
  static uint8_t onCnt = 0, offCnt = 0;
  static bool    state = false;          // 현재 LED 상태

  if (!state) {                          // 현재 OFF
    if (ratio >= TH_ON_RATIO) {
      if (++onCnt >= TH_FRAMES) {        // 조건 충족 → ON
        state = true;  onCnt = 0;
        digitalWrite(PIN_LED1, HIGH);
        digitalWrite(PIN_LED2, HIGH);
      }
    } else onCnt = 0;
  } else {                               // 현재 ON
    if (ratio < TH_OFF_RATIO) {
      if (++offCnt >= TH_FRAMES) {       // 조건 충족 → OFF
        state = false; offCnt = 0;
        digitalWrite(PIN_LED1, LOW);
        digitalWrite(PIN_LED2, LOW);
      }
    } else offCnt = 0;
  }

  /* 30 프레임마다 디버그 출력 */
  static uint32_t f = 0;
  if (++f % 30 == 0)
    Serial.printf("#%lu  red=%u  tot=%u  r=%.3f  LED=%s\n",
                  f, red, tot, ratio, state ? "ON" : "OFF");

  delay(30);                              // ≈25–30 fps
}
