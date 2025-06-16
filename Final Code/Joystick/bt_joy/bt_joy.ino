#include <Arduino.h>
#include <NeoSWSerial.h>

// 버튼 핀 정의
#define PORTD_BUTTON_A  0x04  // PD2
#define PORTD_BUTTON_B  0x08  // PD3
#define PORTD_BUTTON_C  0x10  // PD4
#define PORTD_BUTTON_D  0x20  // PD5
#define PORTB_BUTTON_E  0x01  // PB0

#define X_CHANNEL 0x00 // ADC0
#define Y_CHANNEL 0x01 // ADC1

// NeoSWSerial 객체: RX=12, TX=13
NeoSWSerial mySerial(12, 13);

void init_ADC() {
  ADMUX  = (1 << REFS0);               // AVcc reference, ADC0 selected
  ADCSRA = (1 << ADEN)                 // ADC enable
         | (1 << ADPS2)               // Prescaler 128
         | (1 << ADPS1)
         | (1 << ADPS0);
}

int read_ADC(uint8_t channel) {
  channel &= 0x07;
  ADMUX = (ADMUX & 0xF8) | channel;
  ADCSRA |= (1 << ADSC);              // Start conversion
  while (ADCSRA & (1 << ADSC));       // Wait for complete
  return ADC;
}

void setup() {
  // 디버깅용 시리얼
  Serial.begin(9600);
  // HC-05 Bluetooth 통신용
  mySerial.begin(9600);
  init_ADC();

  // 버튼 풀업 설정
  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |=  (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |=  PORTB_BUTTON_E;
}

void loop() {
  // 1) X, Y ADC 값 읽기
  int rawX = read_ADC(X_CHANNEL);
  int rawY = read_ADC(Y_CHANNEL);

  // 2) 버튼 읽기
  char btn[6] = {'0','0','0','0','0','\0'};
  if (!(PIND & PORTD_BUTTON_A)) btn[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) btn[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) btn[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) btn[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) btn[4] = 'E';

  // 3) 버퍼에 문자열 저장
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%d,%d,%s\n", rawX, rawY, btn);

  // 4) NeoSWSerial로 전송
  mySerial.print(buffer);

  // 5) 하드웨어 시리얼로 디버깅 출력
  Serial.print("TX >> ");
  Serial.print(buffer);

  delay(50);
}
