#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 13);  // RX=12, TX=13

void setup() {
  Serial.begin(9600);
  mySerial.begin(38400);  // HC-05 송신기와 동일한 속도
}

void loop() {
  if (mySerial.available()) {
    String line = mySerial.readStringUntil('\n');
    line.trim();

    Serial.print("Raw line: [");
    Serial.print(line);
    Serial.println("]");

    int firstComma  = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      String sX = line.substring(0, firstComma);
      String sY = line.substring(firstComma + 1, secondComma);
      String sBtn = line.substring(secondComma + 1);

      sX.trim(); sY.trim(); sBtn.trim();

      Serial.print("sX: ["); Serial.print(sX); Serial.println("]");
      Serial.print("sY: ["); Serial.print(sY); Serial.println("]");
      Serial.print("sBtn: ["); Serial.print(sBtn); Serial.println("]");

      int rawX = sX.toInt();
      int rawY = sY.toInt();
      String buttons = sBtn;

      Serial.print("RX >> X: "); Serial.print(rawX);
      Serial.print("  Y: ");     Serial.print(rawY);
      Serial.print("  Buttons: "); Serial.println(buttons);
    }
  }
}
