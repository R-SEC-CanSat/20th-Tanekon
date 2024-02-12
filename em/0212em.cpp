//#include <M5Core2.h>
//標準ライブラリ
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
//それ以外のライブラリ
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);//BNO055の設定
//三鷹ファミマの緯度経度
double goalGPSdata[2] = {35.700993, 139.566498};
//三鷹駅の緯度経度
double goalGPSdata2[2] = {35.702797, 139.561109};
//ゴールの緯度経度(作業場近くのセブン)
double goalGPSdata3[2] = {35.717147,139.823209};

//カメラの値を受け取るためのシリアル通信
SoftwareSerial mySerial(9, 10);    // RX,TXの割り当て
String pre_camera_data[2];
int camera_data[3];

//PID制御のための定数
#define Kp 1
const int STBY = 2;     // モータードライバの制御の準備
const int AIN1 = 3;     // 1つ目のDCモーターの制御
const int AIN2 = 4;     // 1つ目のDCモーターの制御
const int BIN1 = 7;     // 2つ目のDCモーターの制御
const int BIN2 = 8;     // 2つ目のDCモーターの制御
const int PWMA = 5;     // 1つ目のDCモーターの回転速度
const int PWMB = 6;    // 2つ目のDCモーターの回転速度
const int fusePin = 9;
const int RESET_PIN = 10;
const int pps_PIN = 11;

double currentGPSdata[3];
double eulerdata[3];
double azidata[2];

//I2C communication parameters
#define DEFAULT_DEVICE_ADDRESS 0x42
TwoWire& gps = Wire;

//I2C read data structures
char buff[32];
int idx = 0;

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


volatile bool ppsTriggered = false;


void ppsHandler(void);


void ppsHandler(void)
{
  ppsTriggered = true;
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

//Read 32 bytes from I2C
void readI2C(char *inBuff)
{
  gps.requestFrom((uint8_t)DEFAULT_DEVICE_ADDRESS, (uint8_t) 32);
  int i = 0;
  while (gps.available()) {
    inBuff[i] = gps.read();
    i++;
  }
}


//緯度経度の取得
void GPS(){
    
    //While the message isn't complete
    while (!ppsTriggered) {
    char c ;
    if (idx == 0) {
        readI2C(buff);
        delay(1);
    }
    //Fetch the character one by one
    c = buff[idx];
    idx++;
    idx %= 32;
    //If we have a valid character pass it to the library
    if ((uint8_t) c != 0xFF) {
        Serial.print(c);
        nmea.process(c);
    }
    }
    if (ppsTriggered) {
    ppsTriggered = false;
    long latitude_mdeg = nmea.getLatitude();//小数点以下6桁を小数点無しで出してるのでしてるので注意
    long longitude_mdeg = nmea.getLongitude();
    double latitude_deg = latitude_mdeg / 1000000.0;
    double longitude_deg = longitude_mdeg / 1000000.0;
    double goaldirection = 57.2957795131 * atan2(goalGPSdata2[0] - latitude_deg, goalGPSdata2[1] - longitude_deg);//ラジアンで出てる  
    //北を0度とした0~360の値に変換
    if(goaldirection > 90 && goaldirection < 180){
        goaldirection = 450 - goaldirection;
    }
    else{
        goaldirection = 90 - goaldirection;
    }
    currentGPSdata[0] = latitude_deg;
    currentGPSdata[1] = longitude_deg;
    currentGPSdata[2] = goaldirection;
    nmea.clear();
    }
}

//オイラー角の取得、四元数から取ってる
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
    Euler();
    Serial.println(eulerdata[1]);
    while (1)
    {if (eulerdata[1] <= 70) {
        digitalWrite(fusePin, HIGH); // 溶断回路を通電
        delay(1000);
        digitalWrite(fusePin, LOW); // 
        break;
        }
    else{
        delay(1000);
        }
    }

}
void GetAzimuthDistance(){
    GPS();
    Euler();
    //回転の程度をとりあえず整えてみる(turnpower∈[-180,180])
    double turnpower;
    turnpower = currentGPSdata[2] - eulerdata[2];
    if (turnpower > 180 && turnpower < 360){
        turnpower = turnpower - 360;
    }
    else if(turnpower < -180 && turnpower > -360){
        turnpower = turnpower + 360;
    }
    else if(turnpower < 0 && turnpower > -180){
        turnpower = turnpower;
    }
    else{
        turnpower = turnpower;
    }
    Serial.print("GPS Data : ");
    Serial.print(currentGPSdata[2]);
    Serial.print("\tEuler Data: ");
    Serial.println(eulerdata[2]);
    Serial.print("\tMoterControl : ");
    Serial.println(turnpower);
    
    azidata[0] =  Kp * turnpower;
    azidata[1] = sqrt(pow(goalGPSdata2[0] - currentGPSdata[0], 2) + pow(goalGPSdata2[1] - currentGPSdata[1], 2));
    Serial.print("\tDistance: ");
    Serial.println(azidata[1]);

}
//GPSとオイラー角から右回転を正として回転量を出す
void P_GPS_Moter(){
    Serial.println("P_GPS_Moter");
    while(true){
    GetAzimuthDistance();
    if(azidata[1] < 5){
        break;
        }
    else{
        int PID_left = 0.7 * azidata[0] + 126;
        int PID_right = - 0.7 * azidata[0] + 126;
        MoterControl(PID_left, PID_right);
        delay(250);
        }
    } 
}

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
    for(int i = 0; i < 3; i++){
        camera_data[i] = pre_camera_data[i].toInt();
    }
    
}
void P_camera_Moter(){
  char buff[255];
  int counter = 0;
  while(mySerial.available()>0){
    char val = char(mySerial.read());
    buff[counter] = val;
    counter++;  
    if (val == '\n'){
        Serial.println(buff);
        //文字列を整数リストに変換
        split(buff);
        if(camera_data[2]>70){
            break;
        }
        Serial.print(camera_data[0]);
        Serial.print(",");
        Serial.println(camera_data[1]);
        int PID2_left = 0.75 * (camera_data[0]-160) + 120;
        int PID2_right = 0.75 * (160 - camera_data[0]) + 120;
        Serial.print("\tMoterControl: ");
        Serial.println(PID2_left);
        Serial.println(PID2_right);

        MoterControl(PID2_left,PID2_right);
        counter = 0;
        pre_camera_data[0] = "";
        pre_camera_data[1] = "";
    }
  }
}

void setup(void)
{
  Serial.begin(9600);
  mySerial.begin(57600);// ソフトウェアシリアル通信の開始
  delay(1000);

  Wire.begin();

    //dcモーター関連
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    digitalWrite(STBY, HIGH); // モータードライバ制御準備
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(fusePin, OUTPUT);

    //BNO055関連
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(1000);
    //Start the module
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, HIGH);
    Serial.println("Resetting GPS module ...");
    gpsHardwareReset();
    Serial.println("... done");

    delay(1000);

    //Reinitialize I2C after the reset
    gps.begin();

    //clear i2c buffer
    char c;
    idx = 0;
    memset(buff, 0, 32);
    do {
    if (idx == 0) {
        readI2C(buff);
        delay(1);
    }
    c = buff[idx];
    idx++;
    idx %= 32;
    }
    while ((uint8_t) c != 0xFF);

    pinMode(11, INPUT);
    attachInterrupt(digitalPinToInterrupt(11), ppsHandler, RISING);
}
void loop() {
    delay(2000);
    housyutu();
    P_GPS_Moter();
    P_camera_Moter();
}