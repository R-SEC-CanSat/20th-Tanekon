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

#include <SPI.h> /*SPIヘッダーをインクルード*/
#define MAXCS 19 /*SSピンをGPIO19に*/
static const int spiClk = 5500000; /*クロックを1MHzに*/
SPIClass * thermo_spi = NULL /*SPIクラスを格納するポインタを準備*/

void setup(){
    thermo_spi = new SPIClass(VSPI); /*インスタンスを生成*/
    thermo_spi -> begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, MAXCS); /*指定したピンでSPI通信を確立*/
    pinMode(MAXCS, OUTPUT); /*begin()でSSピンを指定しても自動でピンモードを設定してくれないので手動で設定する.*/
}

/*通信コード*/
void measurement(){
    /* beginTransaction()でSPI通信の開始を宣言して，
     * digitalWrite()でSSピンを手動で操作する．
     * transfer()でデータの送受信を行い，
     * digitalWrite()でSSピンを元に戻してから，
     * endTransaction()でSPI通信を終了する．       */
    thermo_spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
    digitalWrite(MAXCS, LOW);
    /*今回はMAX31855に合わせて32bit通信用関数を使用．
     *もちろん普通の8bit通信のtransfer()を4回呼び出してもいい．
     *MAX31855は受信機能はないので，送るデータは何でもいい．   */
    receive_data = thermo_spi->transfer32(0x32241608);
    digitalWrite(MAXCS, HIGH);
    thermo_spi->endTransaction();
}
