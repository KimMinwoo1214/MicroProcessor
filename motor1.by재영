/*
// PWM 폭(us)
volatile int roll_us     = 1500;
volatile int pitch_us    = 1500;
volatile int throttle_us = 1000;
volatile int yaw_us      = 1500;

String inputString = "";

#define ROLL_PIN     9    
#define PITCH_PIN    10   
#define THROTTLE_PIN 3    
#define YAW_PIN      5    

void setup() {
  Serial.begin(9600);

  pinMode(ROLL_PIN, OUTPUT);
  pinMode(PITCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(YAW_PIN, OUTPUT);                                                       //완성 후 DDRx, PORTx 로 바꾸기
  

  // Timer1 설정: 0.5us 해상도로 CTC 모드
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  TCCR1B |= (1 << WGM12);  // CTC 모드
  TCCR1B |= (1 << CS11);   // 프리스케일러 8 → 2MHz → 0.5us 해상도
  OCR1A = 40000;           // 20ms = 40000 × 0.5us
  TIMSK1 |= (1 << OCIE1A); 
  sei();
}

void loop() {
  // 블루투스 문자열 수신
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseCommand(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

// 20ms마다 PWM 4채널 출력 (인터럽트) 
ISR(TIMER1_COMPA_vect) {
  
  digitalWrite(ROLL_PIN, HIGH);
  digitalWrite(PITCH_PIN, HIGH);
  digitalWrite(THROTTLE_PIN, HIGH);
  digitalWrite(YAW_PIN, HIGH);

  delayMicroseconds(roll_us);
  digitalWrite(ROLL_PIN, LOW);

  delayMicroseconds(pitch_us - roll_us);                                                     //완성 후 DDRx, PORTx 로 바꾸기
  digitalWrite(PITCH_PIN, LOW);

  delayMicroseconds(throttle_us - pitch_us);ss
  digitalWrite(THROTTLE_PIN, LOW);

  delayMicroseconds(yaw_us - throttle_us);
  digitalWrite(YAW_PIN, LOW);
}

// 문자열                                                                   //민우가 한 블루투스 확인해보고 함수 바꾸기, 보낸 데이터가 어떤 형식인지 같이 이야기
void parseCommand(String data) {
  data.trim();
  if (data.length() < 8) return;

  int tIdx = data.indexOf('T');
  int yIdx = data.indexOf('Y');
  int rIdx = data.indexOf('R');
  int pIdx = data.indexOf('P');

  if (tIdx != -1 && yIdx != -1)
    throttle_us = constrain(data.substring(tIdx + 1, yIdx).toInt(), 1000, 2000);
  if (yIdx != -1 && rIdx != -1)
    yaw_us = constrain(data.substring(yIdx + 1, rIdx).toInt(), 1000, 2000);
  if (rIdx != -1 && pIdx != -1)
    roll_us = constrain(data.substring(rIdx + 1, pIdx).toInt(), 1000, 2000);
  if (pIdx != -1)
    pitch_us = constrain(data.substring(pIdx + 1).toInt(), 1000, 2000);
}        */

  if (pIdx != -1)
    pitch_us = constrain(data.substring(pIdx + 1).toInt(), 1000, 2000);
}

