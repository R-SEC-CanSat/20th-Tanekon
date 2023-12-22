#include <Arduino.h>

typedef struct{
    int in1;
    int in2;
    int pwm;
} MotorPin;

void MotorOut(MotorPin pin[], int speed, int omega){
    //モーターの速度調整
    int speed_R,speed_L;
    if(speed + omega > 255 || speed - omega < 255){
        speed = speed * (255 - omega) / abs(speed);
    }
    speed_R = speed + omega;
    speed_L = speed - omega;

    int MotorSpeed[2] = {speed_R, speed_L};

    //出力
    for(int i = 0; i < 2; i++){
        if(MotorSpeed[i] > 0){
            digitalWrite(pin[i].in1, HIGH);
            digitalWrite(pin[i].in2, LOW);
            analogWrite(pin[i].pwm, MotorSpeed[i]);
        }else if(MotorSpeed[i] < 0){
            digitalWrite(pin[i].in1, LOW);
            digitalWrite(pin[i].in2, HIGH);
            analogWrite(pin[i].pwm, -MotorSpeed[i]);
        }else{
            digitalWrite(pin[i].in1, LOW);
            digitalWrite(pin[i].in2, LOW);
            analogWrite(pin[i].pwm, 0);
        }
    }
}

void setup() {
    // モーターのピンを設定
    MotorPin motorPin[2] = {
        {2, 3, 4}, // モーター右
        {5, 6, 7}  // モーター左
    };

    // ピンを出力モードに設定
    for(int i = 0; i < 2; i++){
        pinMode(motorPin[i].in1, OUTPUT);
        pinMode(motorPin[i].in2, OUTPUT);
        pinMode(motorPin[i].pwm, OUTPUT);
    }
}

void loop() {
    // モーターの速度と角速度を設定
    int speed = 200; // この値はモーターの速度によります
    int omega = 50; // この値はモーターの角速度によります

    // モーターを制御
    MotorOut(motorPin, speed, omega);
}
