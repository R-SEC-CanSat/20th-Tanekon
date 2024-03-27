#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test");
  Serial.println("device start");
  pinMode(13, OUTPUT);
  pinMode(15, INPUT_PULLUP);
}

char databox;
void loop() {
  if (digitalRead(15) == LOW) {
    Serial.println("LED is ON.");
    SerialBT.write('T');
  }
  if (digitalRead(15) == HIGH) {
    Serial.println("LED is OFF.");
    SerialBT.write('L');
  }

  

  if (SerialBT.available()) {
    databox = SerialBT.read();
    Serial.println(databox);

    if (databox == 'L') {
      digitalWrite(13, LOW);
    }
    if (databox == 'T') {
      digitalWrite(13, HIGH);
    }
  }
  delay(20);
}