#include <SPI.h>
#include <SD.h>
#include <RTClib.h>

const int chipSelect = 10; // SDカードモジュールのチップ選択ピン
RTC_DS3231 rtc; // RTCモジュールのインスタンス

void setup() {
  Serial.begin(9600);
  
  // SDカードの初期化
  if (!SD.begin(chipSelect)) {
    Serial.println("SDカードの初期化に失敗しました");
    return;
  }
  
  // RTCモジュールの初期化
  if (!rtc.begin()) {
    Serial.println("RTCの初期化に失敗しました");
    return;
  }

  // CSVファイルのヘッダーを作成
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("時刻,シーケンス,外部モジュールの有無,赤コーンまでの距離");
    dataFile.close();
    Serial.println("CSVファイルのヘッダーを作成しました");
  } else {
    Serial.println("CSVファイルのヘッダー作成に失敗しました");
  }
}

void loop() {
  // 時刻を取得
  DateTime now = rtc.now();

  // シーケンス、外部モジュールの有無、赤コーンまでの距離を仮の値として定義（センサーからの値を取得する必要あり）
  int sequence = 1;
  boolean externalModule = true;
  float distanceToRedCone = 10.5;

  // CSVファイルにデータを書き込む
  writeDataToCSV(now, sequence, externalModule, distanceToRedCone);

  delay(1000); // 1秒待つ
}

void writeDataToCSV(DateTime timestamp, int sequence, boolean externalModule, float distanceToRedCone) {
  // CSVファイルを開く
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    // 時刻、シーケンス、外部モジュールの有無、赤コーンまでの距離を書き込む
    dataFile.print(timestamp.timestamp()); // 時刻はUnixタイムスタンプで保存
    dataFile.print(",");
    dataFile.print(sequence);
    dataFile.print(",");
    dataFile.print(externalModule);
    dataFile.print(",");
    dataFile.println(distanceToRedCone);
    dataFile.close();
    Serial.println("データをCSVファイルに書き込みました");
  } else {
    Serial.println("CSVファイルの書き込みに失敗しました");
  }
}
