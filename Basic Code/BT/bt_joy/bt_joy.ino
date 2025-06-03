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
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC enable, Prescaler 128
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
  char buttons[5];
};

void setup() {
  Serial.begin(9600);
  mySerial.begin(38400);
  init_ADC();

  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |= (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |= PORTB_BUTTON_E;
}

void loop() {
  int X = read_ADC(X_CHANNEL);
  int Y = read_ADC(Y_CHANNEL);

  char buttons[5] = {'0', '0', '0', '0', '0'};
  if (!(PIND & PORTD_BUTTON_A)) buttons[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) buttons[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) buttons[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) buttons[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) buttons[4] = 'E';


  DataPacket dataPacket = { X, Y, {buttons[0], buttons[1], buttons[2], buttons[3], buttons[4]} };

  mySerial.write((uint8_t*)&dataPacket, sizeof(dataPacket));

  Serial.print("X: "); Serial.print(X);
  Serial.print(" Y: "); Serial.print(Y);
  Serial.print(" Buttons: "); Serial.println(buttons);

  delay(50);
}
