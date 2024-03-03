//etoe前に確認すること
//１．溶断のピン番号と時間
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

//BNO055 setting
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double setGPSdata[2] = {0,0};
double currentGPSdata[3] = {0,0,0};
double eulerdata[3] = {0,0,0};
double azidata[2] = {0,0};

//gps setting
//三鷹ファミマの緯度経度
double goalGPSdata3[2] = {35.70099, 139.56649};
//運河駅
double goalGPSdata2[2] = {35.91435, 139.90586};
//三鷹駅の緯度経度
double goalGPSdata4[2] = {35.70279, 139.56110};
//ゴールの緯度経度(作業場近くのセブン)
double goalGPSdata[2] = {35.717167, 139.823181};
//I2C communication parameters
#define DEFAULT_DEVICE_ADDRESS 0x42
//I2C read data structures
char buff[80];
int idx = 0;
char lat[10],lon[11];
int PID_left;
int PID_right;

//camera setting
SoftwareSerial mySerial;
String pre_camera_data[3];
int camera_data[3];
double camera_area_data = 0.0;

//dcmoter setting
#define gpsTp 0.75
#define gpsSp 120
#define cameraTp 0.65
#define cameraSp 0
const int STBY = 17; // モータードライバの制御の準備
const int AIN1 = 16; // 1つ目のDCモーターの制御
const int AIN2 = 4; // 1つ目のDCモーターの制御
const int BIN1 = 5; // 2つ目のDCモーターの制御
const int BIN2 = 18; // 2つ目のDCモーターの制御
const int PWMA = 0; // 1つ目のDCモーターの回転速度
const int PWMB = 19; // 2つ目のDCモーターの回転速度
const int RESET_PIN = 15;
const int fusePin = 23;  // 溶断回路の制御,今はダミー
//led点灯用


//SDcard setting
const int SD_MOSI = 27;
const int SD_MISO = 25;
const int SD_SCK  = 26;
const int SD_CS_PIN  = 14;

File mainFile;
File subFile;

SPIClass SPISD(HSPI);

//collect module setting
const int  SERVO_PIN = 13;
Servo servo;
void ledmaker(int count){
    for(int i = 0; i < count; i++)
    digitalWrite(fusePin, HIGH); // 溶断回路を通電
    delay(500);
    digitalWrite(fusePin, LOW); // 
    delay(500);
}
void SD_init(){
  //  SPIClass SPI2(HSPI);
    SPISD.begin(SD_SCK, SD_MISO, SD_MOSI);
    if (!SD.begin(SD_CS_PIN,SPISD)) {  //SD_CS_PIN this pin is just the dummy pin since the SD need the input 
    Serial.println(F("failed!"));
    return;
    }
    else Serial.println(F("SD read!"));
    mainFile = SD.open("/main.txt", "a"); //append to file
    subFile = SD.open("/sub.txt", "a"); 
  if (mainFile){
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.close();
    Serial.println("done.");
  }
  else{
    Serial.println("error opening test.txt to write");
  }
  if (subFile){
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.close();
    Serial.println("done.");
  }
  else{
    Serial.println("error opening test.txt to write");
  }

  /*
  myFile = SD.open("/test.txt", "r"); //read from file
  if (myFile)
  {
    Serial.println("test.txt:");
    String inString;  //need to use Strings because of the ESP32 webserver
    while (myFile.available())
    {
      inString += myFile.readString();
    }
    myFile.close();
    Serial.print(inString);
  }
  else
  {
    Serial.println("error opening test.txt to read");
  }
  */
}

//左右の回転速度を0基準に設定(v∈[-255,255])
void MoterControl( int left,int right) {
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
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
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
void GPS_data(){
  
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
                    goaldirection -= 90;
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
                        Serial.println(mlat);
                        Serial.println(mlat.substring(2,4).toDouble());
                        Serial.println(mlat.substring(4,9).toDouble());
                        Serial.println(mlat.substring(6,8).toDouble());
                        double latitude = mlat.substring(0,2).toDouble() + mlat.substring(2,9).toDouble() / 60.0 / 100000.0;
                        String mlon = String(lon);
                        double longitude = mlon.substring(0,3).toDouble() + mlon.substring(3,10).toDouble() / 60.0 / 100000.0;
                        double goaldirection  = 57.2957795131 * atan2(goalGPSdata[0] - latitude, goalGPSdata[1] - longitude);
                        if(goaldirection > -90){
                            goaldirection -= 90;
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
                        Serial.println(mlat.substring(2,4).toDouble());
                        Serial.println(mlat.substring(4,9).toDouble());
                        Serial.println(mlat.substring(6,8).toDouble());
                        double latitude = mlat.substring(0,2).toDouble() + mlat.substring(2,9).toDouble() / 60.0 / 100000.0;
                        String mlon = String(lon);
                        double longitude = mlon.substring(0,3).toDouble() + mlon.substring(3,10).toDouble() / 60.0 / 100000.0;
                        double goaldirection  = 57.2957795131 * atan2(goalGPSdata[0] - latitude, goalGPSdata[1] - longitude);
                        if(goaldirection > -90){
                            goaldirection -= 90;
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
    //オイラー角の値を調整
    eulerdata[2] = yaw -22;
}


void stack(){
    if(eulerdata[0] > 88){
    MoterControl(-255,-255);
    delay(1000);
    stop();
    }
}

void parakaihi(){


}

void kaishuu(){




}
void setting(){


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

void GetAzimuthDistance(){
    GPS_data();
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
    azidata[1] = distanceBetween(goalGPSdata[0],goalGPSdata[1],currentGPSdata[0],currentGPSdata[1]);
    Serial.print("\tDistance: ");
    Serial.println(azidata[1]);
}

//GPSとオイラー角から右回転を正として回転量を出す
//移動平均法を用いる
void P_GPS_Moter(){ 
    double lat_data[5];
    double lon_data[5];
    double dit_data[5];
    Serial.println("P_GPS_Moter");
    for(int i = 0; i < 5; i++){
        GetAzimuthDistance();
        lat_data[i] = azidata[0];
        lon_data[i] = azidata[1];
        dit_data[i] = azidata[2];
        
        double ave_lat = (lat_data[0] + lat_data[1] + lat_data[2] + lat_data[3] + lat_data[4]) / 5;
        double ave_lon = (lon_data[0] + lon_data[1] + lon_data[2] + lon_data[3] + lon_data[4]) / 5;
        double ave_dit = (dit_data[0] + dit_data[1] + dit_data[2] + dit_data[3] + dit_data[4]) / 5;
    }
    while(true){
        if(azidata[1] < 20){
            break;
            }
        else{
            if(azidata[0] > 0){
                PID_left = gpsSp;
                PID_right = gpsSp - gpsTp * azidata[0];
            }
            else{
                PID_right = gpsSp;
                PID_left = gpsSp + gpsTp * azidata[0];
            }
            Serial.print("\tMoterpower : ");
            Serial.print(PID_left);
            Serial.print(",");
            Serial.println(PID_right);
            MoterControl(PID_left, PID_right);
            delay(250);
            
        } 
        
    }
}
void housyutu(){
    Serial.println("housyutu");
    //横の状態を確認
    while (1){
        Euler();
        Serial.println(eulerdata[1]);
        if (eulerdata[1] <= -45 || eulerdata[1] >= 45) {
            delay(1000);
            break;
            }
        else{
            delay(1000);
            }
    }
    while(1){
        Euler();
        Serial.println(eulerdata[1]);
        if (eulerdata[1] <= 45 && eulerdata[1] >= -45) {
            delay(1000);
            Serial.println("youdan");
            digitalWrite(fusePin, HIGH); // 溶断回路を通電
            delay(500);
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
    //文字列データの初期化
    pre_camera_data[0] = "";
    pre_camera_data[1] = "";
    pre_camera_data[2] = "";
    for (int i = 0; i < 15; i++) {
        char tmp = data.charAt(i);
        if (tmp == '$') {
            startParsing = true;
            continue; // Skip processing the '$' character
        }
        
        if (startParsing) {
            //画角に赤が写っていない場合の例外処理
            if((index == 0) && (tmp == '0')){
                pre_camera_data[0] = "0";
                pre_camera_data[1] = "0";
                pre_camera_data[2] = "0.0";
                break;
            }
            if (tmp == ',') {
                index++;
            } else {
                pre_camera_data[index] += tmp;
            }

        }
    }
    String prebottom_area_data = String(pre_camera_data[2]);
    Serial.println(prebottom_area_data);
    String bottom_area_data = prebottom_area_data.substring(2,6);
    Serial.println(bottom_area_data);
    //文字列リストを整数リストに変換
    for(int i = 0; i < 2; i++){
        camera_data[i] = pre_camera_data[i].toInt();
    }
    camera_area_data = bottom_area_data.toDouble() / 10000.0;
    Serial.println(camera_area_data);
    
    
}
void P_camera_Moter(){
    char buff[255];
    int counter = 0;
    while(1){//カメラによる制御のためのループ
        while (mySerial.available()) { // 同期のためにデータをすべて一旦破棄する
            mySerial.flush();
        }
        delay(1);
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
                        if (counter == 15){
                            Serial.println(buff);
                            //文字列を整数リストに変換
                            split(buff);
                            if(camera_area_data>0.40){
                                break;
                            }
                            Serial.print(camera_data[0]);delay(10);//シリアルモニタの表示がバグるので時間を置く、ここが変
                            Serial.print(",");delay(10);
                            Serial.println(camera_data[1]);delay(10);
                            int PID2_left = cameraTp * (camera_data[0]-160) + cameraSp;
                            int PID2_right = cameraTp * (160 - camera_data[0]) + cameraSp;
                            MoterControl(PID2_left,PID2_right);
                            Serial.print("\tMoterControl: ");delay(10);
                            Serial.print(PID2_left);delay(10);
                            Serial.print(",");delay(10);
                            Serial.println(PID2_right);delay(10);

                            //MoterControl(PID2_left,PID2_right);
                            delay(1000);
                            counter = 0;
                            break;
                        }
                    }
                }
                if(camera_area_data>0.40){//二重ループを抜けるための応急措置
                    break;
                }
            }
        }
    }
}
void gpsHardwareReset()
{
  //reset the device
  digitalWrite(RESET_PIN, LOW);
  delay(50);
  digitalWrite(RESET_PIN, HIGH);

  //wait for reset to apply
  delay(2000);

}


void setup() {
    //serial setting
    Serial.begin(57600);
    // 速度、RX、TX、?、?、バッファ
    mySerial.begin(57600,32, 33,SWSERIAL_8N1,false,256);
    Serial.println("Starting ...");
    
    //i2c setting
    Wire.begin(21,22);
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
    pinMode(RESET_PIN, OUTPUT);
    pinMode(fusePin, OUTPUT);
    digitalWrite(fusePin, LOW); // 溶断回路を通電

    //gps hardware reset
    Serial.println("Resetting GPS module ...");
    //gpsHardwareReset();
    Serial.println("... done");
    delay(1000);
    // servo setting
    servo.attach(SERVO_PIN,510,2400);
    //BNO055関連
    delay(1000);
    bool status = bno.begin();
    if (!bno.begin()){
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(100);

    //SDcard setting
    SD_init();
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
}
void missionready(){
    //highly
    for(int i = 0; i < 25; i++){
        MoterControl(10 * i,10 * i);
        delay(500);
    }
    //slowly
    for(int i = 0; i < 25; i++){
        MoterControl(250 - 10 * i,250 - 10 * i);
        delay(500);
    }
    stop();
    GPS_data();
    setGPSdata[0] = currentGPSdata[0];
    setGPSdata[1] = currentGPSdata[1];

}
void loop() {
    //release sequence
    Serial.println("release sequence start");
    delay(100);
    housyutu();
    missionready();
    P_GPS_Moter();
    ledmaker(2);
    //アーム展開
    Serial.println("camera sequence start");
    P_camera_Moter();
    //delay(1000);
    delay(9999999);
    // put your main code here, to run repeatedly:
}