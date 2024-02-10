// Arduino Nano
const int STBY = 2;     // モータードライバの制御の準備
const int AIN1 = 19;     // 1つ目のDCモーターの制御
const int AIN2 = 27;     // 1つ目のDCモーターの制御
//const int BIN1 = 6;     // 2つ目のDCモーターの制御
//const int BIN2 = 7;     // 2つ目のDCモーターの制御
const int PWMA = 25;     // 1つ目のDCモーターの回転速度
//const int PWMB = 10;    // 2つ目のDCモーターの回転速度
void setup() {
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    //pinMode(BIN1, OUTPUT);
    //pinMode(BIN2, OUTPUT);
    digitalWrite(STBY, HIGH); // スタンバイ
    pinMode(PWMA, OUTPUT);
    //pinMode(PWMB, OUTPUT);
}
void loop() {
    // 前進（後進は全て逆にする）
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    //digitalWrite(BIN1, HIGH);
    //digitalWrite(BIN2, LOW);
    Serial.println("P_GPS_Moter");
    delay(1000);
    // 右回り（左回りは逆）
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    //digitalWrite(BIN1, HIGH);
    //digitalWrite(BIN2, LOW);
    Serial.println("P_GPS_Moter");
    delay(1000);
    // 回転速度を設定（0～255）まで
    analogWrite(PWMA, 50);
    //analogWrite(PWMB, 50);
}
