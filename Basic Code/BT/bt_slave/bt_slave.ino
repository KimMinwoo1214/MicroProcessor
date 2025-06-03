#include <SoftwareSerial.h>
#include <Arduino.h>

// SoftwareSerial 객체: RX=10, TX=11 (TX는 필요시만 연결)
SoftwareSerial mySerial(12, 13);

// 수신한 데이터를 저장할 버퍼 크기
#define RX_BUFFER_SIZE 32
char rxBuffer[RX_BUFFER_SIZE];

void setup() {
  Serial.begin(9600);       // 디버깅용 하드웨어 시리얼
  mySerial.begin(38400);     // 송신부와 동일한 9600bps
}

void loop() {
  // 1) 한 줄 단위로 받을 때까지 기다리기
  //    readStringUntil('\n')를 사용하거나, 아래처럼 직접 처리 모드도 가능
  if (mySerial.available()) {
    // mySerial.readBytesUntil() 등을 사용해도 되지만,
    // 여기서는 readStringUntil() 형태로 간단히 구현
    String line = mySerial.readStringUntil('\n'); // '\n' 전까지 읽고 버림

    // 개행 문자('\n')가 떨어지기 전까지 데이터가 쌓였을 때만 파싱 시도
    if (line.length() > 0) {
      // 2) 문자열 예시: "334,582,A0C00"
      //    콤마(,)로 구분하여 rawX, rawY, buttons 추출
      int firstComma  = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        // rawX: 0 ~ firstComma-1까지 서브스트링
        String sX = line.substring(0, firstComma);
        // rawY: firstComma+1 ~ secondComma-1까지
        String sY = line.substring(firstComma + 1, secondComma);
        // buttons: secondComma+1 ~ 끝까지
        String sBtn = line.substring(secondComma + 1);

        int rawX = sX.toInt();   // 정수로 변환
        int rawY = sY.toInt();   // 정수로 변환
        String buttons = sBtn;   // 예: "A0C00"

        // 3) 디버깅용 출력
        Serial.print("RX >> X: "); Serial.print(rawX);
        Serial.print("  Y: ");       Serial.print(rawY);
        Serial.print("  Buttons: "); Serial.println(buttons);

        // 4) 여기에서 rawX, rawY, buttons를 이용한 제어 로직 추가
        //    예: 모터 속도 결정, LED 토글, 서보 제어 등
      }
    }
  }

  delay(10);
}
