#include <Arduino.h>

typedef struct {
    double w;
    double x;
    double y;
    double z;
} quaternion;

typedef struct {
    double roll;
    double pitch;
    double yaw;
} euler_angles;

euler_angles quat_to_euler(quaternion quat) {
    euler_angles result;

    // roll (x-axis rotation)
    double t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z);
    double t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y);
    result.roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    result.pitch = asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y);
    double t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);  
    result.yaw = atan2(t3, t4);

    result.roll *= 57.2957795131;
    result.pitch *= 57.2957795131;
    result.yaw *= 57.2957795131;

    return result;
}

void setup() {
    Serial.begin(9600);

    // テスト用のクォータニオンを設定
    quaternion test_quat;
    test_quat.w = 1.0;
    test_quat.x = 0.0;
    test_quat.y = 0.0;
    test_quat.z = 0.0;

    // クォータニオンをオイラー角に変換
    euler_angles result = quat_to_euler(test_quat);

    // 結果を出力
    Serial.print("Roll: ");
    Serial.println(result.roll);
    Serial.print("Pitch: ");
    Serial.println(result.pitch);
    Serial.print("Yaw: ");
    Serial.println(result.yaw);
}

void loop() {
    
}