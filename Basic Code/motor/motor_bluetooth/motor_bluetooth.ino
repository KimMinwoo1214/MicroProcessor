#include <SoftwareSerial.h>
#include <Arduino.h>

// PWM 핀 정의
#define ROLL_PIN     11
#define PITCH_PIN    10
#define THROTTLE_PIN 6
#define YAW_PIN      9
#define AUX_PIN     3  // ARM 신호용

SoftwareSerial mySerial(A2, A3); // HC-05 연결 (RX=12, TX=13)

int rollPWM     = 1500;
int pitchPWM    = 1500;
int throttlePWM = 885;
int yawPWM      = 1000;
int auxPWM      = 1000;

int pwmStep     = 10;
bool armState   = false;
bool lastBtnEState = false;

unsigned long lastPWMTime = 0;
const int pwmPeriod = 20000; // 20ms 주기 (μs)

void setup() {
  pinMode(ROLL_PIN, OUTPUT);
  pinMode(PITCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(YAW_PIN, OUTPUT);
  pinMode(AUX_PIN, OUTPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);

  Serial.begin(9600);
  mySerial.begin(38400);
}

void loop() {
  if (mySerial.available()) {
    String line = mySerial.readStringUntil('\n');
    line.trim();

    int arrowPos = line.indexOf(">>");
    if (arrowPos != -1) {
      line = line.substring(arrowPos + 2);
      line.trim();
    }

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);

    if (c1 > 0 && c2 > c1) {
      int rawX = line.substring(0, c1).toInt();
      int rawY = line.substring(c1 + 1, c2).toInt();
      String btn = line.substring(c2 + 1);
      btn.trim();

      // 버튼 A/B → Pitch 조절
      if (btn.indexOf('A') != -1) pitchPWM += pwmStep;
      if (btn.indexOf('B') != -1) pitchPWM -= pwmStep;

      // 버튼 C/D → Throttle 조절
      if (btn.indexOf('C') != -1) throttlePWM += pwmStep;
      if (btn.indexOf('D') != -1) throttlePWM -= pwmStep;

      // ✅ 버튼 E → 토글 방식 ARM 제어
      bool currentBtnE = btn.indexOf('E') != -1;
      if (currentBtnE && !lastBtnEState) {
        armState = !armState;  // 상태 반전
        Serial.print("[ARM TOGGLE] -> ");
        Serial.println(armState ? "ON" : "OFF");
      }
      lastBtnEState = currentBtnE;

      // 조이스틱 → Roll/Yaw
      if (rawX < 333) rollPWM = map(rawX, 0, 333, 1000, 1500);
      else            rollPWM = map(rawX, 333, 675, 1500, 2000);

      if (rawY < 333) yawPWM = map(rawY, 0, 333, 1000, 1500);
      else            yawPWM = map(rawY, 333, 675, 1500, 2000);

      rollPWM     = constrain(rollPWM,     1000, 2000);
      pitchPWM    = constrain(pitchPWM,    1000, 2000);
      throttlePWM = constrain(throttlePWM, 1000, 2000);
      yawPWM      = constrain(yawPWM,      1000, 2000);

      auxPWM = armState ? 1600 : 1000;

      Serial.print("ROLL: "); Serial.print(rollPWM);
      Serial.print("  PITCH: "); Serial.print(pitchPWM);
      Serial.print("  THR: "); Serial.print(throttlePWM);
      Serial.print("  YAW: "); Serial.print(yawPWM);
      Serial.print("  AUX: "); Serial.println(auxPWM);
    }
  }

  // 20ms 주기로 PWM 출력
  if (micros() - lastPWMTime >= pwmPeriod) {
    lastPWMTime = micros();
    sendPWM(rollPWM, pitchPWM, throttlePWM, yawPWM, auxPWM);
  }
}

void sendPWM(int roll, int pitch, int throttle, int yaw, int aux) {
  digitalWrite(ROLL_PIN, HIGH);
  digitalWrite(PITCH_PIN, HIGH);
  digitalWrite(THROTTLE_PIN, HIGH);
  digitalWrite(YAW_PIN, HIGH);
  digitalWrite(AUX_PIN, HIGH);

  unsigned long start = micros();
  bool roll_done = false, pitch_done = false;
  bool throttle_done = false, yaw_done = false, aux_done = false;

  while (micros() - start < 2200) {
    unsigned long elapsed = micros() - start;

    if (!roll_done && elapsed >= roll)         { digitalWrite(ROLL_PIN, LOW); roll_done = true; }
    if (!pitch_done && elapsed >= pitch)       { digitalWrite(PITCH_PIN, LOW); pitch_done = true; }
    if (!throttle_done && elapsed >= throttle) { digitalWrite(THROTTLE_PIN, LOW); throttle_done = true; }
    if (!yaw_done && elapsed >= yaw)           { digitalWrite(YAW_PIN, LOW); yaw_done = true; }
    if (!aux_done && elapsed >= aux)           { digitalWrite(AUX_PIN, LOW); aux_done = true; }
  }

  if (!roll_done)     digitalWrite(ROLL_PIN, LOW);
  if (!pitch_done)    digitalWrite(PITCH_PIN, LOW);
  if (!throttle_done) digitalWrite(THROTTLE_PIN, LOW);
  if (!yaw_done)      digitalWrite(YAW_PIN, LOW);
  if (!aux_done)      digitalWrite(AUX_PIN, LOW);
}
