#include <SoftwareSerial.h>
 SoftwareSerial mySerial(9, 10);    // RX,TXの割り当て
void setup(){

  Serial.begin(9600);
  mySerial.begin(57600);            // ソフトウェアシリアル通信の開始(ボーレート9600bps)
  

}

void loop(){
  while(mySerial.available()>0){
    int val = mySerial.read();
    //Serial.readString()
    Serial.println(val);
  }
}