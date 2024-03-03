#include <Arduino.h>
#include<ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
Servo servo1;
Servo servo2;
Servo servo3;

const int STBY = 17; // モータードライバの制御の準備
const int AIN1 = 16; // 1つ目のDCモーターの制御
const int AIN2 = 4; // 1つ目のDCモーターの制御
const int BIN1 = 5; // 2つ目のDCモーターの制御
const int BIN2 = 18; // 2つ目のDCモーターの制御
const int PWMA = 0; // 1つ目のDCモーターの回転速度
const int PWMB = 19; // 2つ目のDCモーターの回転速度
const int s1 = 36;
const int s2 = 13;
const int s3 = 39; 


void setup() {
    servo1.attach(s1);
    servo2.attach(s2);
    servo3.attach(s3);
    servo1.write(88);
    servo2.write(0);
    servo3.write(90);
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    digitalWrite(STBY, HIGH); // スタンバイ
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    // 回転速度を設定（0～255）まで
    analogWrite(PWMA, 100);
    analogWrite(PWMB, 100);
    delay(1000);
    Serial.begin(9600);
}
void loop() {
    servo1.write(0);
    servo2.write(90);
    servo3.write(75);
    Serial.println("1");

    // 前進（後進は全て逆にする）
    //digitalWrite(AIN1, HIGH);
    //digitalWrite(AIN2, LOW);
    //digitalWrite(BIN1, HIGH);
    //digitalWrite(BIN2, LOW);
    //delay(1000);

    servo1.write(88);
    servo2.write(0);
    delay(1000);
    servo3.write(90);
    Serial.println("3");
    delay(1000);

}
