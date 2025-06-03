#include <SoftwareSerial.h>
#include <Arduino.h>

#define PORTD_BUTTON_A  0x04 // PIN2
#define PORTD_BUTTON_B  0x08 // PIN3
#define PORTD_BUTTON_C  0x10 // PIN4
#define PORTD_BUTTON_D  0x20 // PIN5
#define PORTB_BUTTON_E  0x01 // PIN8

#define X_CHANNEL 0x00 // ADC0
#define Y_CHANNEL 0x01 // ADC1

SoftwareSerial mySerial(13, 12); // TX=13, RX=12 for Bluetooth module

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
  return ADC;
}

void setup() {
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
  int rawX = read_ADC(X_CHANNEL);
  int rawY = read_ADC(Y_CHANNEL);

  char btn[6] = {'0','0','0','0','0','\0'};
  if (!(PIND & PORTD_BUTTON_A)) btn[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) btn[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) btn[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) btn[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) btn[4] = 'E';

  // 문자열로 변환하여 전송: "X,Y,A0C00\n" 형식
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%d,%d,%s\n", rawX, rawY, btn);
  mySerial.print(buffer);

  // 디버깅용 시리얼 출력
  Serial.print("TX >> ");
  Serial.print(buffer);

  delay(50);
}
