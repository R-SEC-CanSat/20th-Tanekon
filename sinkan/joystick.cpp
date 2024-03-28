//servo check
//モーターの右左
//キャリブレーションの正確性
//etoe前に確認すること
//１．溶断のピン番号と時間
//sdcardの中身消す、書き込み練習
//microsd
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
//それ以外のライブラリ
#include "BluetoothSerial.h"
#define read_x  4                         //X軸(横軸)
#define read_y  0                         //Y軸(縦軸)
#define read_sw 2                         //スイッチ

int x_axis, y_axis, sw;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.printf("%s - run\n",__func__);
  pinMode(read_x, INPUT);
  pinMode(read_y, INPUT);
  pinMode(read_sw, INPUT_PULLUP);
  pinMode(left, OUTPUT);
  pinMode(up, OUTPUT);
  pinMode(right, OUTPUT);
  pinMode(down, OUTPUT);
}

void loop() {
  x_axis = analogRead(read_x);
  y_axis = analogRead(read_y);
  sw     = digitalRead(read_sw);
  
  Serial.print("x=");
  Serial.print(x_axis);
  Serial.print("　y=");
  Serial.print(y_axis);
  Serial.print("　switch　");
  if(sw == 1){                //スイッチを離したとき
    Serial.println("OFF");
  }else{                     //スイッチを押したとき
    Serial.println("ON");
  }

  delay(500);
}