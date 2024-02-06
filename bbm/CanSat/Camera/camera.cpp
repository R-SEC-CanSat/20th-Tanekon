#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
SoftwareSerial mySerial(9, 10);    // RX,TXの割り当て
String pre_camera_data[2];
int camera_data[2];
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
    
}
void camera_value(){
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
      Serial.print(camera_data[0]);
      Serial.print(",");
      Serial.println(camera_data[1]);
      counter = 0;
      pre_camera_data[0] = "";
      pre_camera_data[1] = "";
    }
    
  }
  
  
  

}
void setup(){

  Serial.begin(9600);
  mySerial.begin(57600);            // ソフトウェアシリアル通信の開始(ボーレート9600bps)
}
void loop(){
  camera_value();
  
}