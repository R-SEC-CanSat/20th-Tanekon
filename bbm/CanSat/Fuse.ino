//試験内容
//一定時間ニクロム線に電圧をかけ、テグスを溶断することと同時にニクロム線が千切れないことを確認する。

#include <M5Core2.h>
#define FUSE_PIN 5

int fuse_time = 1000 //電圧をかける時間(ms)
 
void setup(){
  M5.begin();
  pinMode(FUSE_PIN, OUTPUT);          
}
 
 
void loop(){

    digitalWrite(FUSE_PIN, HIGH);  
    delay(1000);
    digitalWrite(FUSE_PIN, LOW);

    exit()
}