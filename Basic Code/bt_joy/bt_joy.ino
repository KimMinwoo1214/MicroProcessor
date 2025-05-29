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
  char DIR_FBL;
  char DIR_FBR;
  char DIR_LR;
  char Mode;
  int V_Left;
  int V_Right;
  char buttons[5];
};

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  init_ADC();

  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |= (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |= PORTB_BUTTON_E;
}

void loop() {
  int X = read_ADC(X_CHANNEL);
  int Y = read_ADC(Y_CHANNEL);

  int steer = map(X, 0, 1023, -255, 255);
  int speed;
  char mode = 'W';
  char dir_FBL = 'N';
  char dir_FBR = 'N';
  char dir_LR = 'N';

  if (Y > 530) {
    dir_FBL = 'F';
    dir_FBR = 'F';
    speed = map(Y, 530, 1023, 0, 255);
  } else if (Y < 490) {
    dir_FBL = 'B';
    dir_FBR = 'B';
    speed = map(Y, 0, 490, 255, 0);
  } else if (X < 500) {
    dir_FBL = 'B';
    dir_FBR = 'F';
    speed = 220;
  } else if (X > 506) {
    dir_FBL = 'F';
    dir_FBR = 'B';
    speed = 220;
  } else {
    speed = 0;
  }

  if (X > 506) {
    dir_LR = 'R';
  } else if (X < 500) {
    dir_LR = 'L';
  }

  int V_Left, V_Right;

  if (dir_FBL == 'F' && dir_FBR == 'F' && dir_LR == 'R') {
    V_Left = speed;
    V_Right = speed - steer;
  } else if (dir_FBL == 'F' && dir_FBR == 'F' && dir_LR == 'L') {
    V_Left = speed + steer;
    V_Right = speed;
  } else if (dir_FBL == 'B' && dir_FBR == 'B' && dir_LR == 'R') {
    V_Left = speed;
    V_Right = speed - steer;
  } else if (dir_FBL == 'B' && dir_FBR == 'B' && dir_LR == 'L') {
    V_Left = speed + steer;
    V_Right = speed;
  } else if ((dir_FBL == 'F' && dir_FBR == 'B') || (dir_FBL == 'B' && dir_FBR == 'F')) {
    V_Left = speed;
    V_Right = speed;
  } else {
    V_Left = speed;
    V_Right = speed;
  }

  V_Left = constrain(V_Left, 0, 220);
  V_Right = constrain(V_Right, 0, 220);

  char buttons[5] = {'0', '0', '0', '0', '0'};
  if (!(PIND & PORTD_BUTTON_A)) buttons[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) buttons[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) buttons[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) buttons[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) buttons[4] = 'E';

  if (!(PIND & PORTD_BUTTON_D)) {
    mode = 'S';
  }

  DataPacket dataPacket = { dir_FBL, dir_FBR, dir_LR, mode, V_Left, V_Right,
    {buttons[0], buttons[1], buttons[2], buttons[3], buttons[4]} };

  mySerial.write((uint8_t*)&dataPacket, sizeof(dataPacket));

  Serial.print("DIR_FBL: "); Serial.print(dir_FBL);
  Serial.print(" DIR_FBR: "); Serial.print(dir_FBR);
  Serial.print(" DIR_LR: "); Serial.print(dir_LR);
  Serial.print(" V_Left: "); Serial.print(V_Left);
  Serial.print(" V_Right: "); Serial.print(V_Right);
  Serial.print(" Buttons: "); Serial.print(buttons);
  Serial.print(" Mode: "); Serial.println(mode);

  delay(50);
}
