
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
//それ以外のライブラリ
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <ESP32Servo.h>
Servo servo;

//カメラの値を受け取るためのシリアル通信
SoftwareSerial mySerial;
String pre_camera_data[2];
int camera_data[3];


const int STBY = 17; // モータードライバの制御の準備
const int AIN1 = 15; // 1つ目のDCモーターの制御
const int AIN2 = 4; // 1つ目のDCモーターの制御
const int BIN1 = 5; // 2つ目のDCモーターの制御
const int BIN2 = 18; // 2つ目のDCモーターの制御
const int PWMA = 0; // 1つ目のDCモーターの回転速度
const int PWMB = 19; // 2つ目のDCモーターの回転速度
const int  SERVO_PIN = 13;

void split(String data){
    int index = 0; 
    int datalength = data.length();
    
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == ',' ) {
            index++;
        }
        else{pre_camera_data[index] += tmp;}
    }
    //文字列リストを整数リストに変換
    for(int i = 0; i < 2; i++){
        camera_data[i] = pre_camera_data[i].toInt();
    }
    double camera_area_data = pre_camera_data[2].toDouble();
    
}
void P_camera_Moter(){
    char buff[255];
  int counter = 0;
  while(1){
  if(mySerial.available()>0){
    char val = char(mySerial.read());
    Serial.print(val);
    buff[counter] = val;
    counter++;  
    if (val == '\n'){
        Serial.println(buff);
        //文字列を整数リストに変換
        split(buff);
        if(camera_data[2]>0.70){
            break;
        }
        Serial.print(camera_data[0]);
        Serial.print(",");
        Serial.println(camera_data[1]);
        int PID2_left = 0.75 * (camera_data[0]-160) + 120;
        int PID2_right = 0.75 * (160 - camera_data[0]) + 120;
        Serial.print("\tMoterControl: ");
        Serial.print(PID2_left);
        Serial.print(",");
        Serial.println(PID2_right);

        //MoterControl(PID2_left,PID2_right);
        delay(1);
        counter = 0;
    }
      
  }}}
void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(STBY, HIGH);
  // スタンバイ
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  // put your setup code here, to run once:
  servo.attach(SERVO_PIN,510,2400); //サーボモーターの初期化
  Serial.begin(9600);
  // 速度、RX、TX、?、?、バッファ
  mySerial.begin(115200,22, 23,SWSERIAL_8N1,false,256);
}
void loop() {
//アーム展開
P_camera_Moter();
servo.write(90);
delay(500);
//servo.attach(26);
//servo.write(170);
//servo.attach(27);
//servo.write(30);
// 回転速度を設定（0～255）まで
analogWrite(PWMA, 150);
analogWrite(PWMB, 150);
// 前進（後進は全て逆にする）
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, HIGH);
digitalWrite(BIN1, HIGH);
digitalWrite(BIN2, LOW);
delay(5000);
//停止
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, LOW);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, LOW);
//後進
digitalWrite(AIN1, HIGH);
digitalWrite(AIN2, LOW);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, HIGH);
delay(500);
//停止
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, LOW);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, LOW);
//回収
/*
servo.write(70);
delay(500);
servo.attach(26);
servo.write(120);
delay(500);
servo.attach(25);
servo.write(90);
delay(500);
*/
// 右回り（左回りは逆）
digitalWrite(AIN1, HIGH);
digitalWrite(AIN2, LOW);
digitalWrite(BIN1, HIGH);
digitalWrite(BIN2, LOW);
delay(3000);
//停止
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, LOW);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, LOW);
//設置
/*
servo.write(180);
delay(500);
servo.attach(26);
servo.write(170);
servo.attach(27);
servo.write(30);
delay(500);
*/
//後進
digitalWrite(AIN1, HIGH);
digitalWrite(AIN2, LOW);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, HIGH);
delay(1500);
//停止
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, LOW);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, LOW);
/*
//アーム収納
servo.attach(27);
servo.write(70);
delay(500);
servo.attach(26);
servo.write(120);
delay(500);
servo.attach(25);
servo.write(90);
delay(500);
*/
//
delay(9999999);
// put your main code here, to run repeatedly:
}