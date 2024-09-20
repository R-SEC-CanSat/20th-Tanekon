// UART2のインスタンスを作成
HardwareSerial aicam(2);

const int bufferSize = 5;  // 送信するバイト数
uint8_t buffer[bufferSize]; // バイトのバッファ

void setup() {
  Serial.begin(115200);    // 1番目のシリアルポートを115200bpsで初期化
  Serial.println("start"); // シリアルモニタに "start" と表示
  aicam.begin(115200);       // 2番目のシリアルポートを9600bpsで初期化
  Serial.println("aicam started"); // デバッグ用メッセージ
}

void loop() {
  if (aicam.available() >= bufferSize) {  // 5バイト以上のデータがあるかをチェック
    Serial.println("Data available"); // デバッグ用メッセージ
    for (int i = 0; i < bufferSize; i++) {
      buffer[i] = aicam.read();  // aicamデバイスからデータを読み取ってバッファに格納
    }
    Serial.write(buffer, bufferSize);  // 5バイトのデータをシリアルモニタに送信
    Serial.println(); // 新しい行を追加
  } else {
    Serial.println("No data"); // デバッグ用メッセージ
  }
  delay(1000); // 少し待つ
}
