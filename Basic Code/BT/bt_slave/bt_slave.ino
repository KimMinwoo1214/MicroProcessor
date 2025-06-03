#include <SoftwareSerial.h>
#include <Arduino.h>

// SoftwareSerial 객체: RX=10, TX=11 (TX는 연결하지 않아도 됩니다)
SoftwareSerial mySerial(12, 13);

// 한 줄 단위로 받을 때 최대 버퍼 크기
#define RX_BUFFER_SIZE 32
char rxBuffer[RX_BUFFER_SIZE];

void setup() {
  Serial.begin(9600);    // 디버깅용 하드웨어 시리얼
  mySerial.begin(9600);  // 송신부와 동일한 9600bps
}

void loop() {
  // 1) SoftwareSerial로부터 한 줄 단위(‘\n’ 포함)로 읽어들임
  if (mySerial.available()) {
    // '\n' 전에 수신된 문자열 전체를 읽음 (개행 문자는 포함되지 않음)
    String line = mySerial.readStringUntil('\n');

    // 1-1) 혹시 '\r' 등이 남아 있을 수 있으니 trim()으로 앞뒤 공백/캐리지 리턴 제거
    line.trim();

    // 디버깅: 수신된 원본 문자열을 모니터에 출력해보기
    Serial.print("Received raw line: \"");
    Serial.print(line);
    Serial.println("\"");

    // 빈 문자열이 아니면 파싱 시도
    if (line.length() > 0) {
      // 2) 예시로 들어오는 형식: "334,582,A0C00"
      //    콤마(,)로 구분해서 rawX, rawY, buttons 추출
      int firstComma  = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);

      // “첫 번째 쉼표”와 “두 번째 쉼표”가 올바르게 발견되었는지 확인
      if (firstComma > 0 && secondComma > firstComma) {
        // rawX: 0부터 firstComma-1까지
        String sX = line.substring(0, firstComma);
        // rawY: firstComma+1부터 secondComma-1까지
        String sY = line.substring(firstComma + 1, secondComma);
        // buttons: secondComma+1부터 끝까지
        String sBtn = line.substring(secondComma + 1);

        // 3) 정수 변환
        int rawX = sX.toInt();
        int rawY = sY.toInt();
        String buttons = sBtn; // 예: "A0C00"

        // 4) 디버깅용 출력
        Serial.print("Parsed X: "); Serial.print(rawX);
        Serial.print("  Y: ");       Serial.print(rawY);
        Serial.print("  Buttons: "); Serial.println(buttons);

        // 5) 여기에서 rawX, rawY, buttons를 이용한 제어 로직 추가
        //    예: 모터 속도 결정, LED 토글 등
      }
      else {
        // 쉼표 위치가 이상할 때
        Serial.println("  -> Parsing error: 쉼표 위치가 올바르지 않음");
      }
    }
  }

  delay(10);
}
