

/*

#define ROLL_PIN     9
#define PITCH_PIN    10
#define THROTTLE_PIN 3
#define YAW_PIN      5

int roll_us     = 1500;
int pitch_us    = 1500;
int throttle_us = 1000;
int yaw_us      = 1500;

unsigned long pwmStartTime = 0;
bool roll_high     = false;
bool pitch_high    = false;
bool throttle_high = false;
bool yaw_high      = false;

String inputString = "";

void setup() {
  Serial.begin(9600);
  pinMode(ROLL_PIN, OUTPUT);
  pinMode(PITCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);                                                           //완성 후 DDRx, PORTx 로 바꾸기
  pinMode(YAW_PIN, OUTPUT);
}

void loop() {
  // 1. 블루투스 수신
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseCommand(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }

  // 2. non-blocking 방식 PWM
  unsigned long now = micros();

  // 2-1. 20ms 주기마다 PWM 신호 시작
  if (now - pwmStartTime >= 20000) {
    pwmStartTime = now;

    // 모든 채널 HIGH 시작
    digitalWrite(ROLL_PIN, HIGH);
    digitalWrite(PITCH_PIN, HIGH);
    digitalWrite(THROTTLE_PIN, HIGH);                                                       //완성 후 DDRx, PORTx 로 바꾸기
    digitalWrite(YAW_PIN, HIGH);

    roll_high = pitch_high = throttle_high = yaw_high = true;
  }

  // 2-2. 각 채널 개별로 LOW로 전환
  if (roll_high && (now - pwmStartTime >= roll_us)) {
    digitalWrite(ROLL_PIN, LOW);
    roll_high = false;
  }

  if (pitch_high && (now - pwmStartTime >= pitch_us)) {
    digitalWrite(PITCH_PIN, LOW);
    pitch_high = false;
  }

  if (throttle_high && (now - pwmStartTime >= throttle_us)) {
    digitalWrite(THROTTLE_PIN, LOW);
    throttle_high = false;
  }

  if (yaw_high && (now - pwmStartTime >= yaw_us)) {
    digitalWrite(YAW_PIN, LOW);
    yaw_high = false;
  }
}

void parseCommand(String data) {                                                              //민우가 한 블루투스 확인해보고 함수 바꾸기, 보낸 데이터가 어떤 형식인지 같이 이야기
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
}
*/

