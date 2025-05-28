

#define ROLL_PIN     9
#define PITCH_PIN    10
#define THROTTLE_PIN 3
#define YAW_PIN      5

volatile int roll_us     = 1500;
volatile int pitch_us    = 1500;
volatile int throttle_us = 1000;
volatile int yaw_us      = 1500;

String inputString = "";

// 상태 플래그
volatile bool pwm_start_flag = false;

void setup() {
  Serial.begin(9600);

  pinMode(ROLL_PIN, OUTPUT);
  pinMode(PITCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(YAW_PIN, OUTPUT);

  // Timer1 → 20ms마다 PWM 시작
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B |= (1 << WGM12);     // CTC 모드
  TCCR1B |= (1 << CS11);      // 프리스케일 8 (2MHz, 0.5us)
  OCR1A = 40000;              // 20ms
  TIMSK1 |= (1 << OCIE1A);    // 비교 인터럽트 허용
  sei();

  // Timer2 → 0.5us 단위에서 각 채널 LOW로 전환
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM21);     // CTC 모드
  TCCR2B |= (1 << CS21);      // 프리스케일 8 (2MHz, 0.5us)
  OCR2A = 1;                  // 아주 빠르게 반복
  TIMSK2 |= (1 << OCIE2A);
}

void loop() {
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

ISR(TIMER1_COMPA_vect) {
  // 모든 채널 HIGH로 설정
  digitalWrite(ROLL_PIN, HIGH);
  digitalWrite(PITCH_PIN, HIGH);
  digitalWrite(THROTTLE_PIN, HIGH);
  digitalWrite(YAW_PIN, HIGH);
  pwm_start_flag = true;
  TCNT2 = 0;
}

ISR(TIMER2_COMPA_vect) {
  static int count_us = 0;

  if (!pwm_start_flag) return;

  count_us++;

  if (count_us == roll_us) {
    digitalWrite(ROLL_PIN, LOW);
  }
  if (count_us == pitch_us) {
    digitalWrite(PITCH_PIN, LOW);
  }
  if (count_us == throttle_us) {
    digitalWrite(THROTTLE_PIN, LOW);
  }
  if (count_us == yaw_us) {
    digitalWrite(YAW_PIN, LOW);
  }

  if (count_us >= 2000) { // 최대 2ms까지만 사용하고 나머지 시간은 대기
    count_us = 0;
    pwm_start_flag = false;
  }
}

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
