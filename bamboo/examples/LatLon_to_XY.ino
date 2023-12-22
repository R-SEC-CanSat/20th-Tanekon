#include <Arduino.h>

typedef struct {
    double lat;
    double lon;
} geographic;

typedef struct {
    double x;
    double y;
} XY;

XY geographic_to_cartesian_2d(geographic origin, geographic point) {
    double R = 6378.137; // Radius of earth in KM

    double lat1 = origin.lat * M_PI / 180;
    double lon1 = origin.lon * M_PI / 180;
    double lat2 = point.lat * M_PI / 180;
    double lon2 = point.lon * M_PI / 180;

    double x = R * (lon2 - lon1) * cos(lat1);
    double y = R * (lat2 - lat1);

    cartesian_2d result;
    result.x = x;
    result.y = y;

    return result;
}

void setup() {
    Serial.begin(9600);

    // テスト用の地理的座標を設定
    geographic machine = {35.681236, 139.767125}; // 東京駅
    geographic RED_Corn = {35.658581, 139.745433}; // 東京タワー

    // 地理的座標を直交座標に変換
    XY result = geographic_to_cartesian_2d(machine, RED_Corn);

    // 結果を出力
    Serial.print("X: ");
    Serial.println(result.x);
    Serial.print("Y: ");
    Serial.println(result.y);
}

void loop() {
    // 何もしない
}