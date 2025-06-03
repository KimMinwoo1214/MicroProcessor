#include <SoftwareSerial.h>
#include <Arduino.h>

// 버튼 핀 정의
#define PORTD_BUTTON_A  0x04 // PD2
#define PORTD_BUTTON_B  0x08 // PD3
#define PORTD_BUTTON_C  0x10 // PD4
#define PORTD_BUTTON_D  0x20 // PD5
#define PORTB_BUTTON_E  0x01 // PB0

// ADC 채널 정의
#define X_CHANNEL 0x00 // ADC0
#define Y_CHANNEL 0x01 // ADC1

SoftwareSerial mySerial(13, 12); // TX=13, RX=12 for Bluetooth module

// DataPacket: X, Y(두 개의 int) + buttons[5]
struct DataPacket {
  int  X;          // ADC raw X value
  int  Y;          // ADC raw Y value
  char buttons[5]; // 'A','B','C','D','E' 중 눌린 버튼 문자 또는 '0'
};

void init_ADC() {
  ADMUX = (1 << REFS0); // AVcc 레퍼런스
  ADCSRA = (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // ADC 활성화, 분주비 128
}

int read_ADC(uint8_t channel) {
  channel &= 0x07;
  ADMUX = (ADMUX & 0xF8) | channel;  // MUX 비트 설정
  ADCSRA |= (1 << ADSC);             // 변환 시작
  while (ADCSRA & (1 << ADSC));      // 변환 완료 대기
  return ADC;                        // 변환된 10비트 값 리턴 (0~1023, 본 예제에서는 0~684 범위)
}

void setup() {
  Serial.begin(9600);    // 디버깅용 시리얼
  mySerial.begin(38400);  // 블루투스 통신 속도
  init_ADC();

  // 버튼 풀업 설정 (active low)
  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |=  (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |=  PORTB_BUTTON_E;
}

void loop() {
  // 1) ADC에서 X, Y 원시값 읽기
  int rawX = read_ADC(X_CHANNEL);
  int rawY = read_ADC(Y_CHANNEL);

  // 2) 버튼 상태 읽어서 배열에 저장
  char btn[5] = {'0', '0', '0', '0', '0'};
  if (!(PIND & PORTD_BUTTON_A)) btn[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) btn[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) btn[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) btn[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) btn[4] = 'E';

  // 3) DataPacket 구조체에 담기
  DataPacket packet;
  packet.X = rawX;
  packet.Y = rawY;
  for (int i = 0; i < 5; i++) {
    packet.buttons[i] = btn[i];
  }

  // 4) 블루투스로 전송 (총 2+2+5 = 9 바이트)
  mySerial.write((uint8_t*)&packet, sizeof(packet));

  // 5) (선택) 디버깅용 시리얼 출력
  Serial.print("Tx — X: "); Serial.print(rawX);
  Serial.print("  Y: ");   Serial.print(rawY);
  Serial.print("  Buttons: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(packet.buttons[i]);
  }
  Serial.println();

  delay(50);
}
