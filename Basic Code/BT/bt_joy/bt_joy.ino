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

// 실제 ADC 값이 0 ~ 684 범위만 나오고, 중앙(중립)이 334라고 하였으므로
const int CENTER = 334;     // 가만히 있을 때의 조이스틱 중앙값
const int MAX_ADC = 684;    // 조이스틱이 읽히는 최대값
const int DEADZONE = 10;    // ±10 범위를 중립으로 인식

void init_ADC() {
  ADMUX = (1 << REFS0); // AVcc reference
  ADCSRA = (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // ADC 활성화, 분주비 128
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
  mySerial.begin(38400);
  init_ADC();

  // 버튼 풀업 설정
  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |=  (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |=  PORTB_BUTTON_E;
}

void loop() {
  int X = read_ADC(X_CHANNEL);
  int Y = read_ADC(Y_CHANNEL);

  // (선택) 디버깅용: ADC 원시값 출력
  Serial.print("ADC X = "); Serial.print(X);
  Serial.print("  ADC Y = "); Serial.println(Y);

  // 초기 상태
  int steer = 0;
  int speed = 0;
  char mode    = 'W';
  char dir_FBL = 'N';  // 전/후진 방향 (앞왼쪽 바퀴)
  char dir_FBR = 'N';  // 전/후진 방향 (앞오른쪽 바퀴)
  char dir_LR  = 'N';  // 좌/우 회전

  // 1) 전진(F) / 후진(B) 판정 — 오로지 Y 기준 (중앙 = 334, 데드존 ±10)
  if (Y > CENTER + DEADZONE) {
    dir_FBL = 'F';
    dir_FBR = 'F';
    // (CENTER+DEADZONE ~ MAX_ADC)를 (0 ~ 255)로 매핑
    speed = map(Y, CENTER + DEADZONE, MAX_ADC, 0, 255);
  }
  else if (Y < CENTER - DEADZONE) {
    dir_FBL = 'B';
    dir_FBR = 'B';
    // (0 ~ CENTER-DEADZONE)를 (255 ~ 0)으로 매핑
    speed = map(Y, 0, CENTER - DEADZONE, 255, 0);
  }
  else {
    dir_FBL = 'N';
    dir_FBR = 'N';
    speed = 0;
  }

  // 2) 좌/우 회전(dir_LR) 및 steer 값 계산 — 오로지 X 기준, 단 속도가 0보다 클 때만
  if (speed > 0) {
    if (X > CENTER + DEADZONE) {
      dir_LR = 'R';
      // (CENTER+DEADZONE ~ MAX_ADC)를 (0 ~ 255)로 매핑
      steer = map(X, CENTER + DEADZONE, MAX_ADC, 0, 255);
    }
    else if (X < CENTER - DEADZONE) {
      dir_LR = 'L';
      // (0 ~ CENTER-DEADZONE)를 (255 ~ 0)으로 매핑
      steer = map(X, 0, CENTER - DEADZONE, 255, 0);
    }
    else {
      dir_LR = 'N';
      steer = 0;
    }
  }
  else {
    dir_LR = 'N';
    steer = 0;
  }

  // 3) 모터 PWM 값 계산 (V_Left, V_Right)
  int V_Left  = 0;
  int V_Right = 0;

  if (dir_FBL == 'F' && dir_FBR == 'F') {
    // 전진 중일 때
    if (dir_LR == 'R') {
      V_Left  = speed;
      V_Right = speed - steer;
    }
    else if (dir_LR == 'L') {
      V_Left  = speed + steer;
      V_Right = speed;
    }
    else {
      V_Left  = speed;
      V_Right = speed;
    }
  }
  else if (dir_FBL == 'B' && dir_FBR == 'B') {
    // 후진 중일 때
    if (dir_LR == 'R') {
      V_Left  = speed;
      V_Right = speed - steer;
    }
    else if (dir_LR == 'L') {
      V_Left  = speed + steer;
      V_Right = speed;
    }
    else {
      V_Left  = speed;
      V_Right = speed;
    }
  }
  else {
    // 앞서 dir_FBL/dir_FBR가 'N'인 경우 (정지)
    V_Left  = 0;
    V_Right = 0;
  }

  // 4) PWM 값 제한 (0 ~ 220)
  V_Left  = constrain(V_Left,  0, 220);
  V_Right = constrain(V_Right, 0, 220);

  // 5) 버튼 입력 읽기
  char buttons[5] = {'0', '0', '0', '0', '0'};
  if (!(PIND & PORTD_BUTTON_A)) buttons[0] = 'A';
  if (!(PIND & PORTD_BUTTON_B)) buttons[1] = 'B';
  if (!(PIND & PORTD_BUTTON_C)) buttons[2] = 'C';
  if (!(PIND & PORTD_BUTTON_D)) buttons[3] = 'D';
  if (!(PINB & PORTB_BUTTON_E)) buttons[4] = 'E';

  // D 버튼 누르면 mode = 'S'
  if (!(PIND & PORTD_BUTTON_D)) {
    mode = 'S';
  }

  // 6) 데이터 패킷 전송
  DataPacket dataPacket = {
    dir_FBL,
    dir_FBR,
    dir_LR,
    mode,
    V_Left,
    V_Right,
    { buttons[0], buttons[1], buttons[2], buttons[3], buttons[4] }
  };
  mySerial.write((uint8_t*)&dataPacket, sizeof(dataPacket));

  // 7) 상태 모니터링 (필요 없으면 주석 처리)
  Serial.print("DIR_FBL: "); Serial.print(dir_FBL);
  Serial.print(" DIR_FBR: "); Serial.print(dir_FBR);
  Serial.print(" DIR_LR: ");  Serial.print(dir_LR);
  Serial.print(" V_Left: ");  Serial.print(V_Left);
  Serial.print(" V_Right: "); Serial.print(V_Right);
  Serial.print(" Buttons: "); Serial.print(buttons);
  Serial.print(" Mode: ");    Serial.println(mode);

  delay(50);
}
