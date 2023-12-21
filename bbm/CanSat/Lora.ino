\\試験内容
\\データの送信
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); //TX, RX
// (Send and Receive)

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
}

void loop() {
  
  mySerial.println("hello");
  
  delay(20);
}