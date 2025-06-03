#include <SoftwareSerial.h>
#include <Arduino.h>

// 버튼 핀 정의
#define PORTD_BUTTON_A  0x04  // PD2
#define PORTD_BUTTON_B  0x08  // PD3
#define PORTD_BUTTON_C  0x10  // PD4
#define PORTD_BUTTON_D  0x20  // PD5
#define PORTB_BUTTON_E  0x01  // PB0

#define X_CHANNEL 0x00 // ADC0
#define Y_CHANNEL 0x01 // ADC1

// SoftwareSerial 객체: TX=13, RX=12
SoftwareSerial mySerial(13, 12);

void init_ADC() {
  ADMUX = (1 << REFS0); // AVcc reference
  ADCSRA = (1 << ADEN)
         | (1 << ADPS2)
         | (1 << ADPS1)
         | (1 << ADPS0); // ADC enable, Prescaler 128
}

int read_ADC(uint8_t channel) {
  channel &= 0x07;
  ADMUX = (ADMUX & 0xF8) | channel;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC; // 0~1023
}

void setup() {
  Serial.begin(9600);         // 디버깅용 하드웨어 시리얼
  mySerial.begin(38400);       // SoftwareSerial을 9600bps로 설정
  init_ADC();

  // 버튼 풀업 설정 (active low)

  Serial.begin(9600);
  mySerial.begin(38400);
  init_ADC();

  // 버튼 입력 풀업 설정
  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |=  (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |=  PORTB_BUTTON_E;
}

void loop() {
  // 1) X, Y ADC 값 읽기
  int rawX = read_ADC(X_CHANNEL);
  int rawY = read_ADC(Y_CHANNEL);

  // 2) 버튼 읽기 (버튼별로 눌리면 문자, 아니면 '0')
  //    마지막에 '\0'을 넣어 문자열로 안전하게 사용
  char btn[6] = {'0','0','0','0','0','\0'};
  if (!(PIND & PORTD_BUTTON_A)) btn[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) btn[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) btn[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) btn[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) btn[4] = 'E';


  // 3) buffer에 "X,Y,버튼문자열\n" 형식으로 저장
  //    예: "334,582,A0C00\n"
  char buffer[32];
  // snprintf를 써서 형식 지정
  snprintf(buffer, sizeof(buffer), "%d,%d,%s\n", rawX, rawY, btn);

  // 4) SoftwareSerial로 전송
  mySerial.print(buffer);

  // 5) 디버깅용으로 하드웨어 시리얼에도 출력

  // 문자열로 변환하여 전송: "X,Y,A0C00\n" 형식
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%d,%d,%s\n", rawX, rawY, btn);
  mySerial.print(buffer);

  // 디버깅용 시리얼 출력
  Serial.print("TX >> ");
  Serial.print(buffer);

  delay(50);
}
