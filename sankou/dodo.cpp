

typedef struct {
    double latitude;
    double longitude;
} location;

typedef struct{
    double x;
    double y;
    double z;
    double w;
} quaternion;

typedef struct {
    double roll;
    double pitch;
    double yaw;
} euler;


typedef struct{
    int8_t IN1_PIN;
    int8_t IN2_PIN;
    int8_t PWM_PIN;
} motorPin;


typedef struct{
    float  ANGLE;
    int    SPEED_ADJUST;
} motorInfo;

motorPin MOTOR_PIN[2];
motorInfo MOTOR_INFO[2];
//モーターAのピン設定
MOTOR_PIN[0].IN1_PIN = 13;
MOTOR_PIN[0].IN2_PIN = 27;
MOTOR_PIN[0].PWM_PIN = 12;
//モーターBのピン設定
MOTOR_PIN[1].IN1_PIN = 4;
MOTOR_PIN[1].IN2_PIN = 2;
MOTOR_PIN[1].PWM_PIN = 15;

const int8_t FUSE_PIN = 5;
//サーボのピン設定
const int8_t SERVO_1 = 25;
const int8_t SERVO_1 = 33;
const int8_t SERVO_1 = 32;
//SDカードのピン設定
const int SD_MOSI = 23;
const int SD_MISO = 19;
const int SD_SCK  = 18;
const int SD_CS  = 5;


//モーターの情報
MOTOR_INFO[0].ANGLE = 0;
MOTOR_INFO[0].SPEED_ADJUST = 0;

MOTOR_INFO[1].ANGLE = 0;
MOTOR_INFO[1].SPEED_ADJUST = 0;

//モーターのPWM設定
ledcAttachPin(MOTOR_PIN[0].PWM_PIN, 0);
ledcSetup(0,12800,8)

ledcAttachPin(MOTOR_PIN[1].PWM_PIN, 1);
ledcSetup(1,12800,8)

//サーボのPWM設定
ledcAttachPin(SERVO_1, 2);
ledcSetup(2,12800,8);
ledcAttachPin(SERVO_2, 3);
ledcSetup(3,12800,8);
ledcAttachPin(SERVO_3, 4);
ledcSetup(4,12800,8);


//使用するピンのアナログピン番号を指定
const int ledPin = A19; 

//pwmの設定。最初の引数がchannel,次が周波数,最後が解像度（ここでは8bit = 256段階）
ledcSetup(0,12800,8); 

//ピンをチャンネルに接続
ledcAttachPin(ledPin,0)


quaternion getQuaternion(){
    imu::Quaternion quat = bno.getQuat();
    quaternion q;
    q.w = quat.w();
    q.x = quat.x();
    q.y = quat.y();
    q.z = quat.z();
    return q;
}

euler Quat_to_Euler(quaternion quat){
    euler gyro;

    double y_sqr = quat.y * quat.y;

    //rollの算出
    double t0 = 2.0 * (quat.w * quat.x + quat.y * quat.z);
    double t1 = 1.0 - 2.0 * (quat.x * quat.x + y_sqr);
    gyro.roll = atan2(t0, t1);

    //pitchの算出
    double t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    gyro.pitch = asin(t2);

    //yawの算出
    double t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y);
    double t4 = +1.0 - 2.0 * (y_sqr + quat.z * quat.z);
    gyro.yaw = atan2(t3, t4);


    //ラジアンから度に変換
    gyro.roll *= 57.2957795131;
    gyro.pitch *= 57.2957795131;
    gyro.yaw *= 57.2957795131;

    //オイラー角の値を調整
    gyro.yaw = yaw - 22;

    return gyro;
}




int motorCalc(MOTOR_INFO MOTOR_INFO[], int speed , int omega , int motorValue[]){
    //モーターの出力値を計算
    for(int i = 0;i<2;i++){
        motorValue[i] = (speed + sign(speed) * MOTOR_INFO[i].SPEED_ADJUST) * cos(MOTOR_INFO[i].ANGLE * PI / 180) + omega;
    }

    //出力が255を超えた場合、speedを減らして255に収まるようにする
    if(abs(motorValue[0]) > 255 || abs(motorValue[1]) > 255){
        motorValue[i] = sign(speed) * (255 - abs(omega)) * cos(MOTOR_INFO[i].ANGLE * PI / 180) + omega;
    }
}




void motorOutput(MOTOR_PIN MOTOR_PIN[], int motorValue[]){
    //モーターの出力値を出力
    for(int i = 0;i<2;i++){
        if(value > 0){
            digitalWrite(motor.IN1, HIGH);
            digitalWrite(motor.IN2, LOW);
            ledcWrite(i, motorValue[i]);
        }
        else if(value < 0){
            digitalWrite(motor.IN1, LOW);
            digitalWrite(motor.IN2, HIGH);
            ledcWrite(i, -motorValue[i]);
        }
        else{
            digitalWrite(motor.IN1, LOW);
            digitalWrite(motor.IN2, LOW);
            ledcWrite(i, 0);
        }
    }
}


//値の符号を返す関数
int sign(int value){
    if(value > 0){
        return 1;
    }
    else if(value < 0){
        return -1;
    }
    else{
        return 0;
    }
}
