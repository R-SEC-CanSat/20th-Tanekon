#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
//bluetooth ready
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String pre_rev_data; //受信データの一時保存用
int rev_data = 0;

void setup() {
    //シリアル通信の設定
    Serial.begin(115200);
    SerialBT.begin("sinkan");
    Serial.println("device start");
    //ピンの設定
    pinMode(23, OUTPUT);
    
}

void loop() {
  if (SerialBT.available()) {
    String databox = SerialBT.readStringUntil(';');
    //受け取った値が1ならLEDを点灯
    if(databox == "1"){
      digitalWrite(23, HIGH);
  }
    //受け取った値が0ならLEDを消灯
    else if(databox == "0"){
      digitalWrite(23, LOW);
    }
    //受け取った値がそれ以外の場合
    else{
      Serial.println(databox);
    }
  }
}