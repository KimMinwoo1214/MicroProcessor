/*
  ESP32-CAM 신호(D9)가 HIGH면 내장 LED(13) ON
  ─ 시리얼 모니터로 핀 상태를 출력해 디버깅 ─
*/
const uint8_t PIN_IN  = 9;        // ESP32-CAM → Uno
//const uint8_t PIN_LED = 13;       // 온보드 LED

void setup() {
  Serial.begin(9600);             // ① 시리얼 초기화 (9600 bps)
  while (!Serial) ;               // (USB CDC가 열릴 때까지 대기·옵션)

  pinMode(PIN_IN, INPUT);         // 외부에서 0/3.3 V 직접 인가
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println(F("=== READY ==="));
}

void loop() {
  int sig = digitalRead(PIN_IN);

  // ② LED 제어
  digitalWrite(LED_BUILTIN, sig);

  // ③ 상태 출력
  Serial.print(F("PIN9="));
  Serial.print(sig);              // 0 또는 1
  Serial.println(sig ? F("  (HIGH)") : F("  (LOW)"));

  delay(200);                     // 0.2 s 간격으로 덜 쏟아지게
}
