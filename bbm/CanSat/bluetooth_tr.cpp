#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String MACadd = "40:f5:20:53:61:9e";
uint8_t address[6]  = {0x40, 0xf5, 0x20, 0x53, 0x61, 0x9e};
bool connected;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test", true); 
  Serial.println("device start");
  
  connected = SerialBT.connect(address);
  if(connected) {
    Serial.println("Connect OK");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("No connect"); 
    }
  }
  
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }

  SerialBT.connect();

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