#include <Arduino.h>
#include <NeoSWSerial.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define PPM_OUT_PIN 10         // OC1A, PPM 출력 핀
#define CHANNELS    5
#define FRAME_US    22500      // 22.5ms PPM 프레임
#define PULSE_US    300        // 채널 시작 LOW 펄스 길이

NeoSWSerial mySerial(A2, A3);  // ← SoftwareSerial → NeoSWSerial

int rollPWM     = 1500;
int pitchPWM    = 1500;
int throttlePWM = 1000;
int yawPWM      = 1500;
int auxPWM      = 1000;

volatile uint16_t ppmValues[CHANNELS] = {1500, 1500, 1000, 1500, 1000};
volatile uint8_t  currentChannel      = 0;
volatile bool     isPulse             = false;
volatile uint16_t totalUs             = 0;

int pwmStep     = 5;
bool armState   = false;
bool lastBtnEState = false;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH); // 확인용

  pinMode(PPM_OUT_PIN, OUTPUT);
  digitalWrite(PPM_OUT_PIN, HIGH); // idle HIGH

  setupPPMTimer();
}

void loop() {
  if (mySerial.available()) {
    String line = mySerial.readStringUntil('\n');
    line.trim();

    int arrow = line.indexOf(">>");
    if (arrow != -1) {
      line = line.substring(arrow + 2);
      line.trim();
    }

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      int rawX = line.substring(0, c1).toInt();
      int rawY = line.substring(c1 + 1, c2).toInt();
      String btn = line.substring(c2 + 1);
      btn.trim();

      // 스로틀 상하 조정
      if (btn.indexOf('A') != -1) throttlePWM += pwmStep;
      if (btn.indexOf('C') != -1) throttlePWM -= pwmStep;

      // YAW는 버튼 누를 때만 특정값, 떼면 복귀
      if (btn.indexOf('B') != -1) yawPWM = 1550;       // 오른쪽
      else if (btn.indexOf('D') != -1) yawPWM = 1450;  // 왼쪽
      else yawPWM = 1500;                              // 중립

      // ARM 토글 버튼 (E)
      bool curE = btn.indexOf('E') != -1;
      if (curE && !lastBtnEState) {
        armState = !armState;
        Serial.print("[ARM TOGGLE] -> ");
        Serial.println(armState ? "ON" : "OFF");
      }
      lastBtnEState = curE;

      if (rawX < 333) rollPWM = map(rawX, 0, 333, 1000, 1500);
      else            rollPWM = map(rawX, 333, 675, 1500, 2000);

      if (rawY < 333) pitchPWM = map(rawY, 0, 333, 1000, 1500);
      else            pitchPWM = map(rawY, 333, 675, 1500, 2000);

      rollPWM     = constrain(rollPWM,     1400, 1600);
      pitchPWM    = constrain(pitchPWM,    1400, 1600);
      throttlePWM = constrain(throttlePWM, 1000, 2000);
      yawPWM      = constrain(yawPWM,      1400, 1600);
      auxPWM      = armState ? 1600 : 1000;

      noInterrupts();
      ppmValues[0] = rollPWM;
      ppmValues[1] = pitchPWM;
      ppmValues[2] = throttlePWM;
      ppmValues[3] = yawPWM;
      ppmValues[4] = auxPWM;
      interrupts();

      Serial.print("ROLL: "); Serial.print(rollPWM);
      Serial.print("  PITCH: "); Serial.print(pitchPWM);
      Serial.print("  THR: "); Serial.print(throttlePWM);
      Serial.print("  YAW: "); Serial.print(yawPWM);
      Serial.print("  AUX: "); Serial.println(auxPWM);
    }
  }
}

void setupPPMTimer() {
  cli(); // 인터럽트 끔

  // CTC 모드, 분주 8
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A  = (F_CPU / 8 / 1000000) * PULSE_US; // 첫 비교: 300µs

  TIMSK1 = (1 << OCIE1A); // 비교 일치 인터럽트 허용

  sei(); // 인터럽트 켬
}

ISR(TIMER1_COMPA_vect) {
  if (isPulse) {
    digitalWrite(PPM_OUT_PIN, HIGH);
    isPulse = false;

    if (currentChannel < CHANNELS) {
      uint16_t dur = ppmValues[currentChannel] - PULSE_US;
      OCR1A = (F_CPU / 8 / 1000000) * dur;
      totalUs += ppmValues[currentChannel];
      currentChannel++;
    } else {
      uint16_t syncUs = FRAME_US - totalUs;
      OCR1A = (F_CPU / 8 / 1000000) * (syncUs - PULSE_US);
      totalUs = 0;
      currentChannel = 0;
    }
  } else {
    digitalWrite(PPM_OUT_PIN, LOW);
    OCR1A = (F_CPU / 8 / 1000000) * PULSE_US;
    isPulse = true;
  }
}
