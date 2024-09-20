#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
//bluetooth ready
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
bool connected;

void setup() {
  //serial ready
  Serial.begin(115200);
  SerialBT.begin("sinkan", true); 
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
  delay(100);

}

void loop() {
  //送信用にデータを整形
    int led = 0;
    String data = String(led)+ ";";
  
  if(led == 1){             
    led = 0;
    Serial.println("OFF");
  }else{                     
    led = 1;
    Serial.println("ON");
  }
  SerialBT.print(data);
  delay(250);
}