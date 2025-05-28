#include <SoftwareSerial.h>
#include <Arduino.h>

#define RX_PIN 12
#define TX_PIN 13

SoftwareSerial bluetoothSerial(RX_PIN, TX_PIN); // RX=12, TX=13 BLUETOOTH MODULE

struct DataPacket {
  char dir;
  int V_Left;
  int V_Right;
  char buttons[5];
};

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);

  Serial.println("Bluetooth communication initialized.");
}

void loop() {
  if (bluetoothSerial.available() >= sizeof(DataPacket)) {
    // Read the data packet
    DataPacket dataPacket;
    bluetoothSerial.readBytes((char *)&dataPacket, sizeof(DataPacket));

    // Print the received data
    Serial.print("Received Dir: ");
    Serial.print(dataPacket.dir);
    Serial.print(" V_Left: ");
    Serial.print(dataPacket.V_Left);
    Serial.print(" V_Right: ");
    Serial.print(dataPacket.V_Right);
    Serial.print(" Buttons: ");
    Serial.print(dataPacket.buttons[0]);
    Serial.print(dataPacket.buttons[1]);
    Serial.print(dataPacket.buttons[2]);
    Serial.println(dataPacket.buttons[3]);
  }
}
