/*******************************************************************
 * ESP32-CAM  : Red Area Detection → GPIO 12 HIGH, GPIO 4 10 % PWM
 * 조건       : 프레임 중 빨강 ≥ 25 % → 12번 100 %, 4번 10 %; 아니면 둘 다 OFF
 *******************************************************************/
#include <Arduino.h>
#include "esp32-hal-ledc.h"     // LEDC 함수 선언
#include "esp_camera.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

/* ───── 사용자 설정 ───── */
#define PIN_FULL   12                 // 항상 FULL 밝기로 쓸 핀
#define PIN_DIM    4                  // 10 % PWM 밝기로 쓸 핀
#define FRAME_SIZE FRAMESIZE_QQVGA    // 160×120
#define RED_RATIO  0.25               // 25 %
/* ────────────────────── */

/* ─── LEDC(PWM) 파라미터 ─── */
const int PWM_CH_DIM = 0;             // 채널 0 → PIN_DIM
const int PWM_FREQ   = 5000;          // 5 kHz
const int PWM_BITS   = 8;             // 8-bit
const int DUTY_DIM   = 26;            // 10 % ≈ 26/255
/* ─────────────────────────── */

static bool isRed(uint16_t rgb)
{
  uint8_t r = ((rgb >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((rgb >>  5) & 0x3F) * 255 / 63;
  uint8_t b = ( rgb        & 0x1F) * 255 / 31;
  return (r > 120) && (r > g + 30) && (r > b + 30);
}

void initPWM()
{
  /* GPIO 12는 부팅 스트랩 핀 → 먼저 LOW로 두고 시작 */
  pinMode(PIN_FULL, OUTPUT);
  digitalWrite(PIN_FULL, LOW);

  ledcSetup(PWM_CH_DIM, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_DIM, PWM_CH_DIM);
  ledcWrite(PWM_CH_DIM, 0);           // 시작 duty = 0
}

void setup()
{
  Serial.begin(115200);
  initPWM();

  /* 카메라 설정 */
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;  cfg.pin_d2 = Y4_GPIO_NUM;
  cfg.pin_d3 = Y5_GPIO_NUM;  cfg.pin_d4 = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM; cfg.pin_pclk = PCLK_GPIO_NUM;
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
  if (!fb) { Serial.println("Frame capture failed"); return; }

  uint16_t *pix = (uint16_t*)fb->buf;
  size_t tot = fb->len >> 1;          // 2 bytes/pixel
  size_t red = 0;

  for (size_t i = 0; i < tot; ++i)
    if (isRed(pix[i])) ++red;
  esp_camera_fb_return(fb);

  bool hit = ((float)red / tot) >= RED_RATIO;

  /* 핀 제어 */
  if (hit) {
    digitalWrite(PIN_FULL, HIGH);     // 12번 FULL 밝기
    ledcWrite(PWM_CH_DIM, DUTY_DIM);  // 4번 10 %
  } else {
    digitalWrite(PIN_FULL, LOW);
    ledcWrite(PWM_CH_DIM, 0);
  }

  /* 디버그: 30 프레임마다 로그 */
  static uint32_t fcnt = 0;
  if (++fcnt % 30 == 0)
    Serial.printf("#%lu  red=%u  tot=%u  ratio=%.3f  -> %s\n",
                  fcnt, red, tot, (float)red / tot,
                  hit ? "ON" : "OFF");

  delay(30);                          // 약 25-30 fps
}
