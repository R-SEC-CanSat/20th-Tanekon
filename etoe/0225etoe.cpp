//etoe前に確認すること
//１．溶断のピン番号と時間
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

//BNO055 setting
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
double currentGPSdata[3] = {0,0,0};
double eulerdata[3] = {0,0,0};
double azidata[2] = {0,0};

//gps setting
//三鷹ファミマの緯度経度
double goalGPSdata2[2] = {35.700993, 139.566498};
//運河駅
double goalGPSdata[2] = {35.914356, 139.905869};
//三鷹駅の緯度経度
double goalGPSdata4[2] = {35.702797, 139.561109};
//ゴールの緯度経度(作業場近くのセブン)
double goalGPSdata3[2] = {35.717147,139.823209};

//camera setting
SoftwareSerial mySerial;
String pre_camera_data[3];
int camera_data[3];
double camera_area_data = 0.0;

//dcmoter setting
const int STBY = 17; // モータードライバの制御の準備
const int AIN1 = 15; // 1つ目のDCモーターの制御
const int AIN2 = 4; // 1つ目のDCモーターの制御
const int BIN1 = 5; // 2つ目のDCモーターの制御
const int BIN2 = 18; // 2つ目のDCモーターの制御
const int PWMA = 0; // 1つ目のDCモーターの回転速度
const int PWMB = 19; // 2つ目のDCモーターの回転速度
const int fusePin = 23;


//collect module setting
const int  SERVO_PIN = 13;
Servo servo;

void Euler(){
    
    imu::Quaternion quat = bno.getQuat();
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();

    double ysqr = y * y;

    // roll (x-axis rotation)
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    double roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch = asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    double yaw = atan2(t3, t4);

    //ラジアンから度に変換
    roll *= 57.2957795131;
    pitch *= 57.2957795131;
    yaw *= 57.2957795131;

    eulerdata[0] = roll;
    eulerdata[1] = pitch;
    eulerdata[2] = yaw;
}
void housyutu(){
    Serial.println("housyutu");
    while (1){
        Euler();
        Serial.println(eulerdata[1]);
        if (eulerdata[1] <= 45 && eulerdata[1] >= -45) {
            Serial.println("youdan");
            digitalWrite(fusePin, HIGH); // 溶断回路を通電
            delay(50000);
            digitalWrite(fusePin, LOW); // 
            break;
            }
        else{
            delay(1000);
            }
    }
}
void split(String data){
    int index = 0; 
    int datalength = data.length();
    bool startParsing = false;
    for (int i = 0; i < 15; i++) {
        char tmp = data.charAt(i);
        if (tmp == '$') {
            startParsing = true;
            continue; // Skip processing the '$' character
        }
        
        if (startParsing) {
            if (tmp == ',') {
                index++;
            } else {
                pre_camera_data[index] += tmp;
            }

        }
    }
    //文字列リストを整数リストに変換
    for(int i = 0; i < 2; i++){
        camera_data[i] = pre_camera_data[i].toInt();
    }
    camera_area_data = pre_camera_data[2].toDouble();
    
}
void P_camera_Moter(){
    char buff[255];
    int counter = 0;
    while(1){//カメラによる制御のためのループ
        if(mySerial.available()>0){
            char val = char(mySerial.read());
            if (val == '$') {
                buff[0] = '$';
                counter++; 
                while(1){//カメラからのシリアル通信によるデータを受け取るためのループ、あとでデータが読めなかった場合の例外を追加する
                    if(mySerial.available()>0){
                        char nextval = char(mySerial.read());
                        buff[counter] = nextval;//x軸、y軸、面積のデータを格納
                        counter++;  
                        //二個目のメッセージであるかを判断
                        if (nextval == '$'){
                            Serial.println(buff);
                            //文字列を整数リストに変換
                            split(buff);
                            if(camera_area_data>0.40){
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
                            delay(1000);
                            counter = 0;
                        }
                    }
                }
            }
        }
    }
}
void setup() {
    //serial setting
    Serial.begin(57600);
    // 速度、RX、TX、?、?、バッファ
    mySerial.begin(57600,32, 33,SWSERIAL_8N1,false,256);
    Serial.println("Starting ...");

    //i2c setting
    Wire.begin();

    //moter setting
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    digitalWrite(STBY, HIGH);
    // スタンバイ
    pinMode(STBY, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(fusePin, OUTPUT);
    digitalWrite(fusePin, LOW);

    // servo setting
    servo.attach(SERVO_PIN,510,2400);
    //BNO055関連
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    
}
void loop() {
    //release sequence
    Serial.println("release sequence start");
    housyutu();
    //アーム展開
    Serial.println("camera sequence start");
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