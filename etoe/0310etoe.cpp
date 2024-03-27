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
#include "SD.h"
#include <SoftwareSerial.h>
//それ以外のライブラリ
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <ESP32Servo.h>
#include <softwareFilter.h>
#include "BluetoothSerial.h"
//misson setting

char progress = 'A';

//BNO055 setting
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double setGPSdata[2] = {0,0};
double currentGPSdata[3] = {0,0,0};
double eulerdata[6] = {0,0,0};
double azidata[2] = {0,999};

//gps setting
//三鷹ファミマの緯度経度
double goalGPSdata3[2] = {35.70099, 139.56649};
//運河駅
double goalGPSdata2[2] = {35.91435, 139.90586};
//三鷹駅の緯度経度
double goalGPSdata4[2] = {35.70279, 139.56110};
//ゴールの緯度経度(作業場近くのセブン)
double goalGPSdata5[2] = {35.717167, 139.823181};
//種子島
double goalGPSdata[2] = {30.3303248, 130.9575781};
//I2C communication parameters
#define DEFAULT_DEVICE_ADDRESS 0x42
//I2C read data structures
char buff[80];
int idx = 0;
char lat[10],lon[11];
int PID_left;
int PID_right;
softwareFilter<double> turn_data(5);
softwareFilter<double> dit_data(5);
double ave_turn;
double ave_dit;

//camera setting
String pre_camera_data[9];
int camera_data[9];
double camera_area_data_red = 0.0;
double camera_area_data_orange = 0.0;
double camera_area_data_yellow = 0.0;

//dcmoter setting
#define gpsTp 0.75
#define gpsSp 200
#define cameraTp 0.5
#define cameraSp 100
const int STBY = 17; // モータードライバの制御の準備
const int AIN1 = 27; // 1つ目のDCモーターの制御
const int AIN2 = 13; // 1つ目のDCモーターの制御
const int BIN1 = 2; // 2つ目のDCモーターの制御
const int BIN2 = 4; // 2つ目のDCモーターの制御
const int PWMA = 12; // 1つ目のDCモーターの回転速度
const int PWMB = 15; // 2つ目のDCモーターの回転速度
const int fusePin = 14;  // 溶断回路の制御,今はダミー
const int s1 = 33;
const int s2 = 32;
const int s3 = 25; 
//led点灯用


//SDcard setting
const int SD_MOSI = 23;
const int SD_MISO = 19;
const int SD_SCK  = 18;
const int SD_CS_PIN  = 5;

File iniFile;
File mainFile;
File subFile;
int fileNum;
char mainName[16];
char subName[16];
char premainName[16];
char presubName[16];

//Bluetooth setting
String device_name = "bamboo";
BluetoothSerial SerialBT;

//collect module setting
Servo servo1;
Servo servo2;
Servo servo3;
bool moduleExist = true;
//光量センサのアナログ値を格納する変数
int cn1Value;

#include <Arduino.h>

String getString(){
  String result;
  result = Serial.readString();
  result.remove(result.length()-2);
  return result;
}



void SD_init(){
  //  SPIClass SPI2(HSPI);
    //SPISD.begin(SD_SCK, SD_MISO, SD_MOSI);
    if (!SD.begin()) {  //SD_CS_PIN this pin is just the dummy pin since the SD need the input 
        Serial.println(F("failed!"));
        return;
    }
    else Serial.println(F("SD read!"));

    // ファイル名決定
    String s1;
    String s2;
    String pres1;
    String pres2;

    while(1){
        
        s1 = "/MAIN";
        s2 = "/SUB";
        if (fileNum < 10) {
            s1 += "00";
            s2 += "00";
        }else if(fileNum < 100) {
            s1 += "0";
            s2 += "0";
        }
        s1 += fileNum;
        s2 += fileNum;
        s1 += ".csv";
        s2 += ".csv";
        s1.toCharArray(mainName, 16);
        s2.toCharArray(subName, 16);
        if(!SD.exists(mainName)){
            pres1 = "/MAIN";
            pres2 = "/SUB";
            if (fileNum < 10) {
                pres1 += "00";
                pres2 += "00";
            }else if(fileNum < 100) {
                pres1 += "0";
                pres2 += "0";
            }
            pres1 += fileNum - 1;
            pres2 += fileNum -1;
            pres1 += ".csv";
            pres2 += ".csv";
            s1.toCharArray(premainName, 16);
            s2.toCharArray(presubName, 16);
            char ch;
            iniFile = SD.open(premainName, FILE_READ);
            if(iniFile){
                while(iniFile.available()){
                ch = iniFile.read();
                Serial.print(ch);
                if(ch >= progress)
                progress = ch;
                }
                iniFile.close();

            } break;
        }
        fileNum++;
        
    }
    mainFile = SD.open(mainName, FILE_WRITE); //append to file
    subFile = SD.open(subName, FILE_WRITE); 
    if (mainFile){
        Serial.print("Writing to test.txt...");
        mainFile.print(progress);
        mainFile.print(',');
        mainFile.print(moduleExist);
        mainFile.print(',');
        mainFile.println(azidata[1]);
        mainFile.close();
        Serial.println("done.");
    }
    else{
        Serial.println("error opening test.txt to write");
    }
    if(progress == 'A'||progress == 'E'){
        
        mainFile = SD.open(mainName, FILE_WRITE); //append to file
        subFile = SD.open(subName, FILE_WRITE); 
        if (mainFile){
            Serial.print("Writing to test.txt...");
            mainFile.print(progress);
            mainFile.print(',');
            mainFile.print(moduleExist);
            mainFile.print(',');
            mainFile.println(azidata[1]);
            mainFile.close();
            Serial.println("done.");
        }
        else{
            Serial.println("error opening test.txt to write");
        }
        if (subFile){
            Serial.print("Writing to test.txt...");
            subFile.println("testing 1, 2, 3.");
            subFile.close();
            Serial.println("done.");
        }
        else{
            Serial.println("error opening test.txt to write");
        }
    }else{
        for(int i = 0; i < 16; i++){
            mainName[i] = premainName[i];
            
        }
    }
}
void SDcard_allremove(){
    for(int i = 0; i < 1000; i++){
        String s1;
        String s2;
        s1 = "/MAIN";
        s2 = "/SUB";
        if (i < 10) {
            s1 += "00";
            s2 += "00";
        }else if(i < 100) {
            s1 += "0";
            s2 += "0";
        }
        s1 += i;
        s2 += i;
        s1 += ".csv";
        s2 += ".csv";
        char mainName[16];
        char subName[16];
        s1.toCharArray(mainName, 16);
        s2.toCharArray(subName, 16);
        SD.remove(mainName);
        SD.remove(subName);
    }
}

//Read 80 bytes from I2C
void readI2C(char *inBuff){
    Wire.requestFrom((uint8_t)DEFAULT_DEVICE_ADDRESS, (uint8_t) 80);
    int i = 0;
    while (Wire.available()) {
        inBuff[i] = Wire.read();
        i++;
    }
}

void setup() {
    //mission setting
    
    //serial setting
    Serial.begin(57600);
    // 速度、RX、TX、?、?、バッファ
    Serial2.begin(57600);
    SerialBT.begin("ESP32test");
    Serial.println("Starting ...");
    //i2c setting
    Wire.begin(21,22);
    delay(100);
    //gps hardware reset
    Serial.println("Resetting GPS module ...");
    Serial.println("... done");
    delay(1000);
    
    //BNO055 setting
    bool status = bno.begin();
    if (!bno.begin()){
        // There was a problem detecting the BNO055 ... check your connections 
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    //clear i2c buffer
    char c;
    idx = 0;
    memset(buff, 0, 80);
    do {
    if (idx == 0) {
        readI2C(buff);
        delay(1);
    }
    c = buff[idx];
    idx++;
    idx %= 80;
    }
    while ((uint8_t) c != 0xFF);
    delay(100);
    
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
    digitalWrite(fusePin, LOW); // 溶断回路を通電
    // servo setting
    servo1.attach(s1,510,2400);
    servo2.attach(s2,510,2400);
    servo3.attach(s3,510,2400);
    servo1.write(88);
    servo2.write(0);
    servo3.write(90);
    //bluetoothsetting
    SerialBT.begin(device_name);
    //SDcard setting
    SD_init();
    
    //デバッグ用進捗タイム
    unsigned long missionstartTime = millis();
}
void printEvent(sensors_event_t* event) {
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        Serial.print("Accl:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else {
    Serial.print("Unk:");
    }
    eulerdata[3] = x;
    eulerdata[4] = y;
    eulerdata[5] = z;
    Serial.print("\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
}
void SD_main_write(String data){
    mainFile = SD.open(mainName, FILE_APPEND); //append to file
    if (mainFile){
        Serial.print("Writing to test.txt...");
        mainFile.print(progress);
        mainFile.print(',');
        mainFile.print(moduleExist);
        mainFile.print(',');
        mainFile.print(azidata[1]);
        mainFile.print(',');
        mainFile.println(data);
        mainFile.close();
        Serial.println("done.");
    }
    else{
        Serial.println("error opening test.txt to write");
    }
    
}
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
    
    sensors_event_t accelerometerData;
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    printEvent(&accelerometerData);
   
    delay(100);
    eulerdata[0] = roll;
    eulerdata[1] = pitch;   
    //オイラー角の値を調整
    eulerdata[2] = yaw + 17.8;

}
void stackcheck_gyaku(){
    
    while(1){
        if(eulerdata[2] > 0){//要調整
            SD_main_write("stack");
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
            analogWrite(PWMA, 255);
            analogWrite(PWMB, 255);
            
        delay(500);
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, LOW);
            analogWrite(PWMA, 0);
            analogWrite(PWMB, 0);
            
        }else{break;}
    }
    
}
//omergaha半時計(右が強くなる)
void MoterControl( int left,int right) {
    //初めにスタック解除
    Euler();

    while(eulerdata[0] > 88){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMA, 255);
        analogWrite(PWMB, 255);
        delay(750);
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, 0);
        analogWrite(PWMB, 0);
        Euler();
    }

    int absleft = abs(left);
    int absright = abs(right);

    if(left >= 0 && right >= 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
    else if(left >= 0 && right < 0){
        digitalWrite(AIN1, HIGH);
    //初めにスタック解除
    Euler();

    while(eulerdata[0] > 88){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMA, 255);
        analogWrite(PWMB, 255);
        delay(750);
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, 0);
        analogWrite(PWMB, 0);
        Euler();
    }

    int absleft = abs(left);
    int absright = abs(right);

    if(left >= 0 && right >= 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
    else if(left >= 0 && right < 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
    else if(left < 0 && right >= 0){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
    else{
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
}
void stop(){
    MotorControl( 0, 0);
}

double distanceBetween(double lat1, double long1, double lat2, double long2){
    // returns distance in meters between two positions, both specified
    // as signed decimal-degrees latitude and longitude. Uses great-circle
    // distance computation for hypothetical sphere of radius 6372795 meters.
    // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    // Courtesy of Maarten Lamers
    double delta = radians(long1-long2);
    double sdlong = sin(delta);
    double cdlong = cos(delta);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double slat1 = sin(lat1);
    double clat1 = cos(lat1);
    double slat2 = sin(lat2);
    double clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * 6372795;
}

void GPS_data_run(double goallat,double goallon){
  
  while(1){
    char c ;
    char* startlonptr = strchr(buff, 'N'); // 指定した文字の位置を取得
    //改行文字で初期化したい
    if (idx == 0 ) {
      readI2C(buff);
      delay(1);
      //Serial.print("readI2C");
      idx = 0;
    }
    //Fetch the character one by one
    c = buff[idx];
    idx++;
    idx %= 80;
    /*
    if (idx %= 100){
        Euler();
        double turnpower;
        turnpower = currentGPSdata[2] - eulerdata[2];
        if (turnpower > 180){
            turnpower -= 360;
        }
        else if (turnpower < -180){
            turnpower += 360;
        }
        Serial.print("GPS Data : ");
        Serial.print(currentGPSdata[2]);
        Serial.print("\tEuler Data: ");
        Serial.print(eulerdata[2]);
        Serial.print("\tMoterpower : ");
        Serial.println(turnpower);
        
        azidata[0] = turnpower;
        azidata[1] = distanceBetween(goallat,goallon,currentGPSdata[0],currentGPSdata[1]);
        Serial.print("\tDistance: ");
        Serial.println(azidata[1]);
        //回転力を更新
        turn_data.dataAdd(azidata[0]);
        //距離を更新
        dit_data.dataAdd(azidata[1]);
        ave_turn = turn_data.filter();
        ave_dit = dit_data.filter();
        if(ave_dit < 20){
            break;
        }
        else{
            if(ave_turn > 0){
                PID_left = gpsSp;
                PID_right = gpsSp - gpsTp * ave_turn;
            }
            else{
                PID_right = gpsSp;
                PID_left = gpsSp + gpsTp * ave_turn;
            }
            Serial.print("\tMoterpower : ");
            Serial.print(PID_left);
            Serial.print(",");
            Serial.println(PID_right);
            MoterControl(PID_left, PID_right);
        } 

    }
    */
    //If we have a valid character pass it to the library
    if ((uint8_t) c != 0xFF) {
      Serial.print(c);
      //GGAならば緯度経度を取得する
      if (c == '$' && idx < 40) {
        if(buff[idx+2] == 'G'){
          if(buff[idx+3] == 'G'){
            if(buff[idx+4] == 'A'){
                for(int i = 0; i < 4; i++){//NMEAフォーマット特有の表記を調整
                    lat[i] = buff[idx+16+i];
                }
                for(int i = 0; i < 5; i++){
                    //小数点は除外する
                    lat[i + 4] = buff[idx+21+i];
                }
                for(int i = 0; i < 5; i++){//NMEAフォーマット特有の表記を調整
                    lon[i] = buff[idx+29+i];
                }
                for(int i = 0; i < 5; i++){
                    //小数点は除外する
                    lon[i + 5] = buff[idx+35+i];
                }

                String mlat = String(lat);
                Serial.println(mlat);
                Serial.println(mlat.substring(2,4).toDouble());
                Serial.println(mlat.substring(4,9).toDouble());
                Serial.println(mlat.substring(6,8).toDouble());
                double latitude = mlat.substring(0,2).toDouble() + mlat.substring(2,9).toDouble() / 60.0 / 100000.0;
                String mlon = String(lon);
                double longitude = mlon.substring(0,3).toDouble() + mlon.substring(3,10).toDouble() / 60.0 / 100000.0;
                double goaldirection  = 57.2957795131 * atan2(goalGPSdata[0] - latitude, goalGPSdata[1] - longitude);
                
                if(goaldirection > -90){
                            goaldirection = goaldirection -90;
                        }else{
                            goaldirection += 270;
                        }
                Serial.print("latitude: ");
                Serial.print(latitude,7);
                Serial.print("\tlongitude: ");
                Serial.println(longitude,7);
                
                delay(10);
                currentGPSdata[0] = latitude;
                currentGPSdata[1] = longitude;
                currentGPSdata[2] = goaldirection;
                break;
              
            }
          }
        }
      }
      //GLLならば緯度経度を取得する
        if (c == '$' && idx < 40) {
            if(buff[idx+2] == 'G'){
                if(buff[idx+3] == 'L'){
                    if(buff[idx+4] == 'L'){
                        for(int i = 0; i < 4; i++){//NMEAフォーマット特有の表記を調整
                            lat[i] = buff[idx+6+i];
                        }
                        for(int i = 0; i < 5; i++){
                            //小数点は除外する
                            lat[i + 4] = buff[idx+11+i];
                        }
                        for(int i = 0; i < 5; i++){//NMEAフォーマット特有の表記を調整
                            lon[i] = buff[idx+19+i];
                        }
                        for(int i = 0; i < 5; i++){
                            //小数点は除外する
                            lon[i + 5] = buff[idx+25+i];
                        }
        
                        String mlat = String(lat);
                        SD_main_write(mlat);
                        Serial.println(mlat);
                        Serial.println(mlat.substring(2,4).toDouble());
                        Serial.println(mlat.substring(4,9).toDouble());
                        Serial.println(mlat.substring(6,8).toDouble());
                        double latitude = mlat.substring(0,2).toDouble() + mlat.substring(2,9).toDouble() / 60.0 / 100000.0;
                        SD_main_write(mlat);
                        String mlon = String(lon);
                        double longitude = mlon.substring(0,3).toDouble() + mlon.substring(3,10).toDouble() / 60.0 / 100000.0;
                        double goaldirection  = 57.2957795131 * atan2(goalGPSdata[0] - latitude, goalGPSdata[1] - longitude);
                                
                        if(goaldirection > -90){
                            goaldirection = goaldirection -90;
                        }else{
                            goaldirection += 270;
                        }
                        Serial.print("latitude: ");
                        Serial.print(latitude,7);
                        Serial.print("\tlongitude: ");
                        Serial.println(longitude,7);
                        
                        delay(10);
                        currentGPSdata[0] = latitude;
                        currentGPSdata[1] = longitude;
                        currentGPSdata[2] = goaldirection;
                        break;
                    
                    }
                }
            }
        }
        //RMCならば緯度経度を取得する
        if (c == '$' && idx < 40) {
            if(buff[idx+2] == 'R'){
                if(buff[idx+3] == 'M'){
                    if(buff[idx+4] == 'C'){
                        for(int i = 0; i < 4; i++){//NMEAフォーマット特有の表記を調整
                            lat[i] = buff[idx+18+i];
                        }
                        for(int i = 0; i < 5; i++){
                            //小数点は除外する
                            lat[i + 4] = buff[idx+23+i];
                        }
                        for(int i = 0; i < 5; i++){//NMEAフォーマット特有の表記を調整
                            lon[i] = buff[idx+31+i];
                        }
                        for(int i = 0; i < 5; i++){
                            //小数点は除外する
                            lon[i + 5] = buff[idx+37+i];
                        }

                        String mlat = String(lat);
                        Serial.println(mlat);
                        Serial.println(mlat.substring(0,2).toDouble());
                        Serial.println(mlat.substring(2,4).toDouble());
                        Serial.println(mlat.substring(4,9).toDouble());
                        double latitude = mlat.substring(0,2).toDouble() + mlat.substring(2,4).toDouble() / 60.0 + mlat.substring(4,9).toDouble() / 60.0 / 100000.0;
                        String mlon = String(lon);
                        Serial.println(mlon);
                        Serial.println(mlon.substring(0,3).toDouble());
                        Serial.println(mlon.substring(3,5).toDouble());
                        Serial.println(mlon.substring(5,11).toDouble());
                        double longitude = mlon.substring(0,3).toDouble() + mlon.substring(3,5).toDouble() / 60.0 + mlon.substring(5,10).toDouble() / 60.0 / 100000.0;
                        double goaldirection  = 57.2957795131 * atan2(goalGPSdata[0] - latitude, goalGPSdata[1] - longitude);
                                
                        if(goaldirection > -90){
                            goaldirection = goaldirection -90;
                        }else{
                            goaldirection += 270;
                        }
                        Serial.print("latitude: ");
                        Serial.print(latitude,7);
                        Serial.print("\tlongitude: ");
                        Serial.println(longitude,7);
                        
                        delay(10);
                        currentGPSdata[0] = latitude;
                        currentGPSdata[1] = longitude;
                        currentGPSdata[2] = goaldirection;
                        break;
                    
                    }
                }
            }
        }
    }
  }
}



void kaishuu(){
    MotorControl(motorPin,150,0);
    delay(2000);
    servo1.write(88);
    servo2.write(0);
    delay(1000);
    servo3.write(90);
    delay(1000);
    //徐々にバック
    for (int i = 0; i < 20; i++){
        MotorControl(motorPin,  - 10 * i,  - 10 * i);
        delay(100);
    }
    //



}
void setting(){
    servo1.write(0);
    delay(1000);
    servo2.write(90);
    delay(1000);
    servo3.write(75);
    delay(1000);
    servo1.write(88);
    delay(1000);
    servo2.write(0);
    delay(1000);
    servo3.write(90);
    delay(1000);
    moduleExist = false;
}

void GetAzimuthDistance(double goallat, double goallon){
    GPS_data_run(goallat, goallon);
    Euler();
    //回転の程度をとりあえず整えてみる(turnpower∈[-180,180])
    double turnpower;
    turnpower = currentGPSdata[2] - eulerdata[2];
    if (turnpower > 180){
        turnpower -= 360;
    }
    else if (turnpower < -180){
        turnpower += 360;
    }
    Serial.print("GPS Data : ");
    Serial.print(currentGPSdata[2]);
    Serial.print("\tEuler Data: ");
    Serial.print(eulerdata[2]);
    Serial.print("\tMoterpower : ");
    Serial.println(turnpower);
    
    azidata[0] = turnpower;
    azidata[1] = distanceBetween(goallat,goallon,currentGPSdata[0],currentGPSdata[1]);
    Serial.print("\tDistance: ");
    Serial.println(azidata[1]);
}

//GPSとオイラー角から右回転を正として回転量を出す
//移動平均法を用いる
void P_GPS_Moter(double goallat2, double goallon2){ 
    Serial.println("P_GPS_Moter");
    
    for(int i = 0; i < 5; i++){
        GetAzimuthDistance(goallat2, goallon2);
        turn_data.dataAdd(azidata[0]);
        dit_data.dataAdd(azidata[1]);
    }
    while(true){    
        GetAzimuthDistance(goallat2, goallon2);
        //回転力を更新
        turn_data.dataAdd(azidata[0]);
        //距離を更新
        dit_data.dataAdd(azidata[1]);
        ave_turn = turn_data.filter();
        int intave_turn = ave_turn;
        ave_dit = dit_data.filter();
        if(ave_dit < 15){
            //徐々に停止
            for (int i = 0; i < 10; i++){
                MotorControl(motorPin, 100 - 10 * i, 100 - 10 * i);
                delay(100);
            }
            break;
        }
        else{
            Serial.print("\tMoterpower : ");
            Serial.print(gpsSp);
            Serial.print(",");
            Serial.println(gpsTp * intave_turn);
            MotorControl(motorPin, gpsSp, gpsTp * intave_turn);
            delay(10);
        } 
        
    }
}

void split(String data){
    int index = 0; 
    int datalength = data.length();
    bool startParsing_R = false;
    bool startParsing_W = false;
    bool startParsing_Y = false;
    //文字列データの初期化
    pre_camera_data[0] = "";
    pre_camera_data[1] = "";
    pre_camera_data[2] = "";
    pre_camera_data[3] = "";
    pre_camera_data[4] = "";
    pre_camera_data[5] = "";
    pre_camera_data[6] = "";
    pre_camera_data[7] = "";
    pre_camera_data[8] = "";
    for (int i = 0; i < data.length(); i++) {
        char tmp = data.charAt(i);
        
        if (startParsing_R) {
            //画角に赤が写っていない場合の例外処理
            if((index == 0) && (tmp == '0')){
                pre_camera_data[0] = "0";
                pre_camera_data[1] = "0";
                pre_camera_data[2] = "0.0";
                startParsing_R = false;
            }
            if (tmp == ',') {
                index++;
                if (index == 3) {
                    startParsing_R = false;
                }
            } else {
                pre_camera_data[index] += tmp;
            }
        }
        if (tmp == 'R') {
            startParsing_R = true;
        }
        
        if (startParsing_W) {
            //画角に白が写っていない場合の例外処理
            if((index == 3) && (tmp == '0')){
                pre_camera_data[3] = "0";
                pre_camera_data[4] = "0";
                pre_camera_data[5] = "0.0";
                startParsing_W = false;
            }
            if (tmp == ',') {
                index++;
                if (index == 6) {
                    startParsing_W = false;
                    
                }
            } else {
                pre_camera_data[index] += tmp;
            }
        }
        if (tmp == 'W') {
            startParsing_W = true;
        }
        
        if (startParsing_Y) {
            //画角に黄が写っていない場合の例外処理
            if((index == 6) && (tmp == '0')){
                pre_camera_data[6] = "0";
                pre_camera_data[7] = "0";
                pre_camera_data[8] = "0.0";
                startParsing_Y = false;
                
            }
            if (tmp == ',') {
                index++;
                if (index == 9) {
                    startParsing_Y = false;
                   break;
                }
            } else {
                pre_camera_data[index] += tmp;
            }
        }
        if (tmp == 'Y') {
            startParsing_Y = true;
            
        }
    }
    //赤色データの変換
    if(pre_camera_data[2] == "0.0"){
        camera_area_data_red = 0.0;
    }else{
        String prebottom_area_data_red = String(pre_camera_data[2]);
        String bottom_area_data_red1 = prebottom_area_data_red.substring(0,1);
        String bottom_area_data_red2 = prebottom_area_data_red.substring(2,6);
        camera_area_data_red = bottom_area_data_red1.toDouble() + bottom_area_data_red2.toDouble() / 10000.0;
    }
    //白色データの変換
    if(pre_camera_data[5] == "0.0"){
        camera_area_data_orange = 0.0;
    }else{
        String prebottom_area_data_orange = String(pre_camera_data[5]);
        String bottom_area_data_orange1 = prebottom_area_data_orange.substring(0,1);
        String bottom_area_data_orange2 = prebottom_area_data_orange.substring(2,6);
        camera_area_data_orange = bottom_area_data_orange1.toDouble() + bottom_area_data_orange2.toDouble() / 10000.0;
    }
    //黄色データの変換
    if(pre_camera_data[8] == "0.0"){
        camera_area_data_yellow = 0.0;
    }else{
        String prebottom_area_data_yellow = String(pre_camera_data[8]);
        String bottom_area_data_yellow1 = prebottom_area_data_yellow.substring(0,1);
        String bottom_area_data_yellow2 = prebottom_area_data_yellow.substring(2,6);
        camera_area_data_yellow = bottom_area_data_yellow1.toDouble() + bottom_area_data_yellow2.toDouble() / 10000.0;
    }
    
    //文字列リストを整数リストに変換
    for(int i = 0; i < 9; i++){
        camera_data[i] = pre_camera_data[i].toInt();
    }
    Serial.println("camera_area_data: ");
    Serial.print(camera_area_data_red);
    Serial.print(",");
    Serial.print(camera_area_data_orange);
    Serial.print(",");
    Serial.println(camera_area_data_yellow);


}

void P_camera_Moter(int colornumber){
    char buff[50];
    int counter = 0;
    char color = 'R'; 
    if (colornumber == 0){
        color = 'R';
    }else if(colornumber == 1){
        color = 'W';
    }else{
        color = 'Y';
    }
    while(1){//カメラによる制御のためのループ
      
        if(Serial2.available()){ // 同期のためにデータをすべて一旦破棄する
            Serial2.flush();
            Serial.println("serial2 flush");
        }
        delay(10);
        if(Serial2.available()){
            char val = char(Serial2.read());
            if (val == 'R') {
                buff[0] = 'R';
                counter++;
                while(1){//カメラからのシリアル通信によるデータを受け取るためのループ、あとでデータが読めなかった場合の例外を追加する
                    if(Serial2.available()>0){
                        char nextval = Serial2.read();
                        buff[counter] = nextval;//x軸、y軸、面積のデータを格納
                        counter++;  
                        //二個目のメッセージであるかを判断
                        if (counter == 50){
                            Serial.println(buff);
                            SD_main_write(String(buff));
                            //文字列を整数リストに変換
                            split(String(buff));
                            Serial.print("camera_data: ");
                            for(int i = 0; i < 9; i++){
                                Serial.print(camera_data[i]);
                                Serial.print(",");
                            }
                            if(camera_area_data_red>0.06 && camera_area_data_red< 1){
                                break;
                            }
                            if(camera_data[colornumber * 3] == 0){
                                MotorControl(motorPin, 0, 100);
                                SD_main_write("no target,turn");
                                delay(100);
                            }else{
                            MotorControl(motorPin, cameraSp, int(cameraTp * (- camera_data[colornumber * 3] + 320)));
                            if((- camera_data[colornumber * 3] + 320)>0){
                                SD_main_write("target,left move");
                            }else{
                                SD_main_write("target,right move");
                            }
                            Serial.print("\tMoterControl: ");delay(10);
                            Serial.print(cameraSp);delay(10);
                            Serial.print(",");delay(10);
                            Serial.println(int(cameraTp * (- camera_data[0] + 320)));delay(10);
                            }
                            delay(1000);
                            counter = 0;
                            break;
                        }
                    }
                }
                if(camera_area_data_red>0.06 && camera_area_data_red< 1){
                    SD_main_write("target,stop");
                    break;
                }
            }
        }
    }
}
void housyutu(){
    Serial.println("housyutu");
    //機体の質量測定
    while (1){
        Euler();
        Serial.println(eulerdata[1]);
        SD_main_write("zyunnbi");
        if (eulerdata[1] <= -45 || eulerdata[1] >= 45) {
            delay(1000);
            break;
            }
        else{
            delay(500);
            }
    }
    //キャリアに入れた後、ゆっくりと反転させる、感覚は狭め
    //横
    while(1){
        Euler();
        Serial.println(eulerdata[1]);
        if (eulerdata[1] <= 45 && eulerdata[1] >= -45) {

            break;
            }
        else{
            delay(500);
            }
    }
    //この状態でクレーンで上げる
    while (1){
        Euler();
        Serial.println(eulerdata[1]);
        SD_main_write("zyunnbi_up");
        if (eulerdata[1] <= -45 || eulerdata[1] >= 45) {
            delay(500);
            break;
            }
        else{
            delay(500);
            }
    }
    //放出
    while(1){
        Euler();
        Serial.println(eulerdata[1]);
        SD_main_write("release");
        if (eulerdata[1] <= 45 && eulerdata[1] >= -45) {

            break;
            }
        else{
            delay(500);
            }
    }

    delay(1000);
}
void tyakuti(){
    unsigned long startTime = millis();
    unsigned long currentTime = millis();
    while(1){
        Euler();
        if(eulerdata[5] < 5){
            SD_main_write("tyakuti with accerelation");
            delay(15000);
            break;
        }
        else if(currentTime - startTime > 15000){
            SD_main_write("tyakuti without accerelation");
            break;
        }
        else{
            currentTime = millis();
        }
        delay(500);
        
    }
    Serial.println("youdan");
    SD_main_write("youdan");
    digitalWrite(fusePin, HIGH); // 溶断回路を通電
    delay(1500);
    digitalWrite(fusePin, LOW); // 
    delay(1000);
}
void missionready(){
    double inilat[5];
    double inilon[5];
    //highly
    SD_main_write("moter ok");
    for(int i = 0; i < 25; i++){
        MotorControl(motorPin, 10 * i, 0);
        delay(500);
    }
    //slowly
    for(int i = 0; i < 25; i++){
        MotorControl(motorPin, 250 - 10 * i, 0);
        delay(500);
    }
    stop();
    SD_main_write("GPS ok");
    GPS_data_run(goalGPSdata[0],goalGPSdata[1]);

    for(int i = 0; i < 5; i++){
        GPS_data_run(goalGPSdata[0],goalGPSdata[1]);
        inilat[i] = currentGPSdata[0];
        inilon[i] = currentGPSdata[1];
    }  
    setGPSdata[0] = (inilat[0] + inilat[1] + inilat[2] + inilat[3] + inilat[4]) / 5;
    setGPSdata[1] = (inilon[0] + inilon[1] + inilon[2] + inilon[3] + inilon[4]) / 5;
    Serial.print("latitude: ");
    Serial.print(setGPSdata[0],7);
    Serial.print("\tlongitude: ");
    Serial.println(setGPSdata[1],7);
    Serial.println("Setting!!!");
    moduleExist = 0;
    SD_main_write("setting");
    setting();
    //back and turn left
    for(int i = 0; i < 25; i++){
        MotorControl(motorPin, -10 * i, 0);
        delay(500);
    }
    MotorControl(motorPin, 0, 100);
    delay(1000);
    for(int i = 0; i < 20; i++){
        MotorControl(motorPin, 10 * i, 0);
        delay(500);
    }
    for(int i = 0; i < 20; i++){
        MotorControl(motorPin, 200 - 10 * i, 0);
        delay(500);
    }
}

void loop() {
    
    //モーター確認徐々に左折と右折
    for(int i = 0; i < 20; i++){
        MotorControl(motorPin, 10 * i, 10 * i);
        delay(100);
    }
    for(int i = 0; i < 20; i++){
        MotorControl(motorPin, 10 * i, -10 * i);
        delay(100);
    }
    //サーボ確認しまってるとき
    servo1.write(0);
    delay(1000);
    servo1.write(88);
    delay(1000);
    servo2.write(90);
    delay(1000);
    //開いてるとき
    servo2.write(0);
    delay(1000);
    servo3.write(0);
    delay(1000);
    servo3.write(90);

    //5回オイラー角を取得して、その平均値を取る
    for (size_t i = 0; i < 5; i++)
    {
        Euler();
        delay(100);
    }
    //commandCheck();
    
    if(progress == 'A'){
        
        //release sequence_1
        Serial.println("release progress start");
        SD_main_write("release");
        delay(100);
        housyutu();
        SD_main_write("housyutu end");
        tyakuti();
        SD_main_write("tyakuti end");
        progress++;
    }
    if(progress == 'B'){
        
        missionready();
        SD_main_write("missionready end");
        //GPS sequence_2
        
        Serial.println("gps progress start");
        SD_main_write("gps start");
        P_GPS_Moter(goalGPSdata[0],goalGPSdata[1]);
        
        //ledmaker(2);
        //アーム展開
        Serial.println("camera progress start");
        SD_main_write("camera start");
        P_camera_Moter(0);
        
        progress++;
    }
    if(progress == 'C'){
        P_GPS_Moter(setGPSdata[0],setGPSdata[1]);
        SD_main_write("setGPSdata end");
        P_camera_Moter(1);
        
        kaishuu();
        
        progress++;
    }
    if(progress == 'D'){
        SD_main_write("gps again start");
        P_GPS_Moter(goalGPSdata[0],goalGPSdata[1]);
        SD_main_write("gps again end");
        SD_main_write("camera again start");
        P_camera_Moter(0);
        SD_main_write("camera again end");
        progress++;
    }
    if(progress == 'E'){
        mainFile = SD.open(mainName, FILE_APPEND); //append to file
        if (mainFile){
            Serial.print("Writing to test.txt...");
            mainFile.print("Misson Complete");
            mainFile.close();
            Serial.println("done.");
        }
        else{
            Serial.println("error opening test.txt to write");
        }
    
    }
    exit(1);
    // put your main code here, to run repeatedly:
}   