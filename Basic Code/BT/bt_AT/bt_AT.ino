#include <SoftwareSerial.h>
SoftwareSerial BTSerial(13, 12); // Rx, Tx (HC-05용)

// 시리얼 통신 설정
void setup() {
  Serial.begin(38400);       // PC와 통신용
  BTSerial.begin(38400);     // HC-05와 통신용 (AT 모드 보레이트
//  pinMode(5, OUTPUT);
}

// 시리얼 데이터 전달
void loop() {
//  digitalWrite(5, HIGH);
  if (BTSerial.available()) {
    Serial.write(BTSerial.read()); // HC-05 → PC
  }
  if (Serial.available()) {
    BTSerial.write(Serial.read()); // PC → HC-05
  }
}
