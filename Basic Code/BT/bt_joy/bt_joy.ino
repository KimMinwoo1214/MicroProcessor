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

struct DataPacket {
  int16_t X;
  int16_t Y;
  char    buttons[5];
};

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600); // SoftwareSerial 속도를 9600으로 낮춤
  init_ADC();

  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |=  (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |=  PORTB_BUTTON_E;
}

void loop() {
  int rawX = read_ADC(X_CHANNEL);
  int rawY = read_ADC(Y_CHANNEL);

  // 버튼 상태를 저장할 때, 마지막에 '\0'을 넣기 위해 6바이트 배열로 선언 후 복사
  char tempBtn[6] = {'0','0','0','0','0','\0'};  
  if (!(PIND & PORTD_BUTTON_A)) tempBtn[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) tempBtn[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) tempBtn[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) tempBtn[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) tempBtn[4] = 'E';

  // DataPacket에 채워 넣을 때는 버튼 5개만, 널 종료문자는 구조체에 포함하지 않음
  DataPacket dataPacket;
  dataPacket.X = (int16_t)rawX;
  dataPacket.Y = (int16_t)rawY;
  for (int i = 0; i < 5; i++) {
    dataPacket.buttons[i] = tempBtn[i];
  }

  // 패킷 전송 (총 2+2+5 = 9바이트)
  mySerial.write((uint8_t*)&dataPacket, sizeof(dataPacket));

  // 디버깅 모니터 출력
  Serial.print("X: "); Serial.print(rawX);
  Serial.print("  Y: "); Serial.print(rawY);
  Serial.print("  Buttons: "); 
  Serial.println(tempBtn);  // 이제 '\0'이 있으므로 안전하게 출력됨

  delay(50);
}
