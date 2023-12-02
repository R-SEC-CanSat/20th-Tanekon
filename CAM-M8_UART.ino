//試験内容
//3m離れた2地点で緯度経度を取得することを10回連続で行う。取得された値が互いに異なることを確認する。
//小数点以下6桁まで取得するように設定
//UART通信を想定

#include <M5Core2.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
 
TinyGPSPlus gps;
SoftwareSerial mySerial(13, 14); // RX, TX
 
void setup() {
 
 Serial.begin(57600);
 while (!Serial) {
 ; 
 }
 
 
 Serial.println("serial_ok");
 
 // set the data rate for the SoftwareSerial port
 mySerial.begin(9600);
 mySerial.println("serial_set");
}
 
void loop() { // run over and over
 while (mySerial.available() > 0){
 char c = mySerial.read();
 //Serial.print(c);
 gps.encode(c);
 if (gps.location.isUpdated()){
 Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
 Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
 Serial.print("ALT="); Serial.println(gps.altitude.meters());
 }
 }
}
