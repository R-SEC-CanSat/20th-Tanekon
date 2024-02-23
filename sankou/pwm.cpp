#include <Arduino.h>
const int ledPin = A4;
 
void setup() {
  // 使用するタイマーのチャネルと周波数を設定
  ledcSetup(0, 12800, 8);
  // ledPinをチャネル0へ接続
  ledcAttachPin(ledPin, 0);
} 
void loop() {
  // 初期の明るさを指定
  static uint8_t brightness = 0;
  // 明るさの変動具合を設定（高いほど急に変わる）
  static int diff = 1;
  // チャネル0に明るさを設定
  ledcWrite(0, brightness);

  // 明るさが0を下回る、若しくは255を超えた時に反転
  if (brightness == 0) {
    diff = 1;
  } else if (brightness == 255) {
    diff = -1;
  }
 
  brightness += diff;
  //遅延を遅くすればゆっくり明るさが変わるようになる
  delay(10);
}