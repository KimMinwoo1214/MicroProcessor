#include <SoftwareSerial.h>
#include <Arduino.h>

// 송신부와 동일한 구조체 정의
struct DataPacket {
  int  X;
  int  Y;
  char buttons[5];
};

SoftwareSerial mySerial(12, 13); // RX=12, TX=13 (TX는 필요 시만 연결)

void setup() {
  Serial.begin(9600);    // 디버깅용 시리얼 모니터
  mySerial.begin(9600);  // 송신부와 동일한 보레이트
}

void loop() {
  // 1) 수신 버퍼에 최소 9바이트 이상 쌓였는지 확인
  if (mySerial.available() >= sizeof(DataPacket)) {
    DataPacket rxPacket;

    // 2) 한꺼번에 읽어서 구조체에 덮어쓰기
    mySerial.readBytes((uint8_t*)&rxPacket, sizeof(rxPacket));

    // 3) 읽어들인 값 확인 (디버깅용)
    Serial.print("Rx — X: "); Serial.print(rxPacket.X);
    Serial.print("  Y: ");    Serial.print(rxPacket.Y);
    Serial.print("  Buttons: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(rxPacket.buttons[i]);
    }
    Serial.println();

    // 4) 실제 처리 로직
    //    예: X, Y 값을 이용해서 모터 속도나 조향을 결정하거나,
    //        버튼 입력에 따라 특정 동작을 수행
    //
    //    예시: 간단히 X, Y 값을 PWM 출력으로 그대로 보낼 경우
    //    analogWrite(LEFT_MOTOR_PIN, map(rxPacket.X, 0, MAX_ADC, 0, 255));
    //    analogWrite(RIGHT_MOTOR_PIN, map(rxPacket.Y, 0, MAX_ADC, 0, 255));
    //
    //    (필요에 따라 map 함수나 제어 로직을 구현하세요.)
  }

  delay(10);
}
