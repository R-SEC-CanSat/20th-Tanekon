import cv2
import numpy as np
import picamera
import time
from smbus import SMBus
import datetime
import csv
import RPi.GPIO as GPIO
import math
from pyproj import Geod
import serial
from micropyGPS import MicropyGPS
import wiringpi as pi
import os


# GPIOピンの定義
AIN1 = 26
AIN2 = 21
PWMA = 19
STBY = 20
BIN1 = 13
BIN2 = 6
PWMB = 5
Led_pin = 11                        #変数"Led_pin"に11を格納

# I2C
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42
i2c = SMBus(1)


def motasetup():
    # GPIOピンの設定
    pi.wiringPiSetupGpio()
    pi.pinMode( AIN1, pi.OUTPUT )
    pi.pinMode( AIN2, pi.OUTPUT )
    pi.pinMode( PWMA, pi.OUTPUT )
    pi.pinMode( STBY, pi.OUTPUT )
    pi.pinMode( BIN1, pi.OUTPUT )
    pi.pinMode( BIN2, pi.OUTPUT )
    pi.pinMode( PWMB, pi.OUTPUT )
    # PWM端子に接続したGPIOをPWM出力できるようにする
    pi.softPwmCreate( PWMA, 0, 100 )
    pi.softPwmCreate( PWMB, 0, 100 )

def bmx_setup():
    # acc_data_setup : 加速度の値をセットアップ
    i2c.write_byte_data(ACCL_ADDR, 0x0F, 0x03)
    i2c.write_byte_data(ACCL_ADDR, 0x10, 0x08)
    i2c.write_byte_data(ACCL_ADDR, 0x11, 0x00)
    time.sleep(0.5)
    # gyr_data_setup : ジャイロ値をセットアップ
    i2c.write_byte_data(GYRO_ADDR, 0x0F, 0x04)
    i2c.write_byte_data(GYRO_ADDR, 0x10, 0x07)
    i2c.write_byte_data(GYRO_ADDR, 0x11, 0x00)
    time.sleep(0.5)
    # mag_data_setup : 地磁気値をセットアップ
    data = i2c.read_byte_data(MAG_ADDR, 0x4B)
    if(data == 0):
        i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
        time.sleep(0.5)
    i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
    i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
    i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
    i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
    i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
    time.sleep(0.5)
       
def acc_value():
    data = [0, 0, 0, 0, 0, 0]
    acc_data = [0.0, 0.0, 0.0]
    try:
        for i in range(6):#各軸に関して2byteずつ存在している
            data[i] = i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i) #1byteよんだら1byte隣に追加
        for i in range(3): #3軸
            acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16 
            if acc_data[i] > 2047: #+-
                acc_data[i] -= 4096
            acc_data[i] *= 0.0098 
    except IOError as e: #例外処理
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return acc_data
    
def gyro_value():
    data = [0, 0, 0, 0, 0, 0]
    gyro_data = [0.0, 0.0, 0.0]
    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(GYRO_ADDR, GYRO_R_ADDR + i)
        for i in range(3):
            gyro_data[i] = (data[2*i + 1] * 256) + data[2*i]
            if gyro_data[i] > 32767:
                gyro_data[i] -= 65536
            gyro_data[i] *= 0.0038
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return gyro_data

def mag_value():
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    mag_data = [0.0, 0.0, 0.0]
    try:
        for i in range(8):
            data[i] = i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)
        for i in range(3):
            if i != 2:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                if mag_data[i] > 4095:
                    mag_data[i] -= 8192
            else:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                if mag_data[i] > 16383:
                    mag_data[i] -= 32768
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return mag_data

def cyakuchi_hantei():
    while True:
        b1=gyro_value()
        time.sleep(0.5)
        b2=gyro_value()
        time.sleep(0.5)
        b3=gyro_value()
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'データ'+','+'ロール角速度'+','+str(b1[0])+'\n')
            file.write(makecontent()+','+'データ'+','+'ロール角速度'+','+str(b2[0])+'\n')
            file.write(makecontent()+','+'データ'+','+'ロール角速度'+','+str(b3[0])+'\n')
        if abs(b1[0])<1.0 and abs(b2[0])<1.0 and abs(b3[0])<1.0:
            cyakuchi=1
            break
        else:
            cyakuchi=0
    return(cyakuchi)

def youdann():
    finish=0
    GPIO.setmode(GPIO.BCM)              #GPIOのモードを"GPIO.BCM"に設定
    GPIO.setup(Led_pin, GPIO.OUT)       #GPIO11を出力モードに設定
    GPIO.output(Led_pin, GPIO.HIGH)     #GPIO11の出力をHigh(3.3V)にする
    time.sleep(2)                       #2秒間待つ
    GPIO.output(Led_pin, GPIO.LOW)      #GPIO11の出力をLow(0V)にする
    GPIO.cleanup()#GPIOをクリーンアップ
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+'GPIO11_high'+'\n')
    finish=1
    return(finish)

def housyutsu():
    with picamera.PiCamera() as camera:
        camera.resolution=(64,48)
        #camera.start_preview()
        #time.sleep(2)
        camera.capture('test.jpg')
    img=cv2.imread('test.jpg')
    img=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    img=np.array(img).flatten()
    mean=img.mean()
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'データ'+','+'照度'+','+str(mean)+'\n')
    print(mean)
    if mean>100:
        #print("housyutsu")
        mean=1
    else:
        #print("mada")
        mean=0
    return(mean)

def housyutsu2():
    while True:
        result1=housyutsu()
        result2=housyutsu()
        result3=housyutsu()
        if result1==1 and result2==1 and result3==1:
            hou=1
            break
        else:
            hou=0
            print("shippai")
    return(hou)

def forward(n):
    # スタンバイ状態にする
    #逆回転真理値
    #print("StandBy")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    print("moveforward")
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100 )
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"前進"+str(n)+"秒"+'\n')

def stop():
    print("Stop!!")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.softPwmWrite( PWMB, 0 )

def stack():
    while True:
        acc = acc_value()
        if acc[2] < 0:
            print("stack! ","acc:",acc[2])
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'データ'+','+"加速度_z"+','+str(acc[2])+'\n')
                file.write(makecontent()+','+'データ'+','+"機体が反転"+'\n')
            forward(2)
            stop()
        else:
            break

def back(n):
    #距離から比例制御してPWMの価を渡す
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 )
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    # print("movefoward")
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100)
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"後退"+str(n)+"秒"+'\n')

def backspin_R(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 ) 
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"右回転"+str(n)+"秒"+'\n')
    
def backspin_L(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 ) 
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    print("backspin_L")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100)
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"左回転"+str(n)+"秒"+'\n')

def offset_mota():
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 ) 
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"右回転10秒"+'\n')

def gazou():
    with picamera.PiCamera() as camera:
        camera.resolution=(64,48)
        #camera.start_preview()
        #time.sleep(2)
        camera.capture('test.jpg')
    img=cv2.imread('test.jpg')
    #画像データの読み込み
    #BGR色空間からHSV色空間への変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #色検出しきい値の設定
    lower = np.array([145,50,0])
    upper = np.array([160,255,255])
    #色検出しきい値範囲内の色を抽出するマスクを作成
    frame_mask = cv2.inRange(hsv, lower, upper)
    # #色検出しきい値の設定
    # lower = np.array([130,64,0])
    # upper = np.array([179,255,255])
    # #色検出しきい値範囲内の色を抽出するマスクを作成
    # frame_mask2 = cv.inRange(hsv, lower, upper)
    # frame_mask = frame_mask1 + frame_mask2
    #論理演算で色検出
    dst = cv2.bitwise_and(img, img, mask=frame_mask)
    #cv.imshow("img", dst)
    cv2.imwrite('noshiro_surface_p1.jpg', dst)
    #画像の読み込み
    img=cv2.imread('noshiro_surface_p1.jpg',0)
    ret1,img_th=cv2.threshold(img,0,255,cv2.THRESH_OTSU)
    #全体の画素数
    whole_area=img_th.size
    #白部分の画素数
    Yellow_area=cv2.countNonZero(img_th)
    #黒部分の画素数
    black_area=whole_area-Yellow_area
    pink = Yellow_area/whole_area*100
    #それぞれの割合を表示
    print('pink_Area='+str(pink)+'%')
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'データ'+','+'パラシュート_割合'+','+str(pink)+'\n')
    return pink
    #print('Black_Area='+str(black_area/whole_area*100)+'%')

def hanntenn():
    stack()
    print("uemuki")

def parakaihi():
    a = acc_value()
    y = gazou()
    kaihi=0
    if a[2] > 3:
        print("uemuki")
        if y >= 7.5: #要調整
            back(5)
            backspin_R(1)
            forward(5)
            stop()
            stack()
            print("mae_para")
            kaihi=1
        else:
            forward(5)
            stop()
            stack()
            print("ushiro_para")
            kaihi=1
    else:
        print("shitamuki")
        hanntenn() 
        if y >= 7.5:
            back(5)
            backspin_R(1)
            forward(5)
            stop()
            stack()
            print("mae_para")
            kaihi=1
        else:
            forward(5)
            stop()
            stack()
            print("ushiro_para")
            kaihi=1

    return(kaihi)            
        
def GPS():
    # シリアル通信設定
    uart = serial.Serial('/dev/serial0', 9600, timeout = 10)
    # gps設定
    my_gps = MicropyGPS(9, 'dd')

    # 10秒ごとに表示
    tm_last = 0
    while True:
        sentence = uart.readline()
        if len(sentence) > 0:
            for x in sentence:
                if 10 <= x <= 126:
                    stat = my_gps.update(chr(x))
                    if stat:
                        tm = my_gps.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 10:
                            #print('=' * 20)
                            #print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                            print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])
                #2個目のforのループを抜けたいから2個目のループの部分でbreak
                #if(my_gps.longitude[0] != 0 and my_gps.latitude[0] != 0):
                #    break
        #最初のwhileループを抜けたいからWhileの部分でbreak。while部分はインデントで下げてるからwhileのましたではなく、whileのtab1つ分下
        if(my_gps.longitude[0] != 0 and my_gps.latitude[0] != 0):
            break
    return my_gps.longitude[0], my_gps.latitude[0]

def get_azimuth_distance():
    #print('=====目標の方位角・距離を取得します=======')

    obj_latitude = 40.14260166666
    obj_longitude = 139.98757166666

    #現在地座標
    p1_longitude, p1_latitude = GPS()
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'データ'+','+'緯度'+','+str(p1_latitude)+'\n')
            file.write(makecontent()+','+'データ'+','+'経度'+','+str(p1_longitude)+'\n')
    #p1_longitude = 135.0
    #p1_latitude = 45.0

    g = Geod(ellps='WGS84')
    # 戻り値は方位角(azimuth)、反方位角(back_azimuth)、距離(distance)の順番
    azimuth, back_azimuth, distance = g.inv(p1_longitude, p1_latitude, obj_longitude, obj_latitude)
    result = g.inv(p1_longitude, p1_latitude, obj_longitude, obj_latitude)
    azimuth = result[0]
    back_azimuth = result[1]
    distance = result[2]
    if(azimuth > -180 and azimuth <0):
        azimuth = azimuth + 360
    return azimuth, distance

def get_theta(azimuth,offset):
    #発散回避した時に変な値が入るのを回避
    theta = 360
    #磁気代入
    mag = mag_value()
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'データ'+','+'磁気_x'+','+str(mag[0])+'\n')
        file.write(makecontent()+','+'データ'+','+'磁気_y'+','+str(mag[1])+'\n')    
    #xy平面のみ見てる
    #arktanの補正
    if(mag[0]-offset[0] != 0):
        if(mag[0]-offset[0] < 0 and mag[1] - offset[1] > 0):
            rad1 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg1 = math.degrees(rad1)
            azimuth_sat = deg1 + 180
        elif(mag[0] - offset[0] < 0 and mag[1] - offset[1] < 0):
            rad2 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg2 = math.degrees(rad2)
            azimuth_sat = deg2 +180
        elif(mag[0]-offset[0] > 0 and mag[1] - offset[1] < 0):
            rad3 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg3 = math.degrees(rad3)
            azimuth_sat = deg3 + 360
        else:
            rad4 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg4 = math.degrees(rad4)
            azimuth_sat = deg4

        #(ii) absが180以上
        if(abs(azimuth-azimuth_sat) > 180):
            theta = 360 - abs(azimuth-azimuth_sat)
        #(i)補正なし
        else:
            theta = abs(azimuth-azimuth_sat)

        print('機体の方位角', azimuth_sat)

        #機体に対するゴールの向きを見る
        #muki、　0:左、1：右
        muki = 0
        theta2 = azimuth_sat - azimuth
        if(theta2 > 0 and abs(theta2) < 180):
            muki = 0
            print('ゴールの向き：左')

        elif(theta2 > 0 and abs(theta2) > 180):
            muki = 1
            print('ゴールの向き：右')

        elif(theta2 < 0 and abs(theta2) < 180):
            muki = 1
            print('ゴールの向き：右')

        elif(theta2 < 0 and abs(theta2) > 180):
            muki = 0
            print('ゴールの向き：左')

        #print('mag', mag)
    if(mag[0]-offset[0] == 0):
        print("分母が0になりました。theta=360を返します")
        #print("magx",mag[0])
        #print("offset_x",offset[0])
        #print("mag-offset",mag[0]-offset[0])
    return theta,muki

def getoffset():
    n11=0
    n12=0
    n13=0
    n22=0
    n23=0
    n1=0
    n2=0
    n3=0
    print ("offset start")
    offset_mota()
    for i in range (200):
        data=mag_value()
        #data=[i-3,i]
        x=data[0]
        y=data[1]
        n11=n11+x**2
        n12=n12+x*y
        n13=n13+x
        n22=n22+y**2
        n23=n23+y
        n1=-(x**3+x*y**2)+n1
        n2=-(x**2*y+y**3)+n2
        n3=-(x**2+y**2)+n3
        time.sleep(0.05)
    stop()
    stack()
    a=np.array([[n11,n12,n13],[n12,n22,n23],[n13,n23,200]])
    #print(a)
    a_inv=np.linalg.inv(a)
    b=np.array([[n1],[n2],[n3]])
    result=np.dot(a_inv,b)
    result_a=result[0][0]
    result_b=result[1][0]
    offset_x=-1*result_a/2
    offset_y=-1*result_b/2
    offset = [offset_x,offset_y]
    print("offset end")
    print(offset_x,offset_y)
    return(offset)

def GPS_yuudou():
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'シーケンス'+','+'オフセット取得開始'+'\n')
    offset = getoffset()
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'データ'+','+'磁気オフセット_x'+str(offset[0])+'\n')
        file.write(makecontent()+','+'データ'+','+'磁気オフセット_y'+str(offset[1])+'\n')
        file.write(makecontent()+','+'シーケンス'+','+'オフセット取得終了'+'\n')
    yuudou=0
    while True:
        acc = acc_value()
        mag = mag_value()
        gyro = gyro_value()
        azimuth, distance = get_azimuth_distance()
        theta, muki = get_theta(azimuth, offset)

        print('ゴールの方位角',azimuth)
        print('ゴールまでの距離',distance,'メートル')
        print('ゴールと機体の方位角差',theta)
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'データ'+','+"方位角差"+","+str(theta)+'\n')
            file.write(makecontent()+','+'データ'+','+"距離"+","+str(distance)+'\n')

        if(distance >= 2):
            if(theta <= 30):
                print('前進')
                forward(5)
                stop()
                stack()

            else:
                if(muki == 0):
                    backspin_L(theta / 180)
                if(muki == 1):
                    backspin_R(theta / 180)
                print('旋回')
                #方位角差によって回転する方向を変えたいが、、、
                #現状差の絶対値しか見れてない
                stop()
                stack()

        else:
            yuudou=1
            break
    return(yuudou)

def gazousyori():
    diffarence=0
    disigion=0
    whiteratio=0
    print("---------------")
    with picamera.PiCamera() as camera:
        camera.resolution = (160, 120)
        #camera.start_preview()
        #time.sleep(2)
        camera.capture('test.jpg')
    img = cv2.imread('test.jpg')
    #img_2=cv2.rotate(img,cv2.ROTATE_180)
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow("HSV", resized_img_HSV)
    low_hsv_min = np.array([0, 100,0])
    low_hsv_max = np.array([5, 255, 255])#画像の2値化（Hueが0近辺）
    maskHSV_low = cv2.inRange(img_HSV,low_hsv_min,low_hsv_max)
    high_hsv_min = np.array([165, 100,0])
    high_hsv_max = np.array([179, 255, 255])#画像の2値化（Hueが179近辺）
    maskHSV_high = cv2.inRange(img_HSV,high_hsv_min,high_hsv_max)#２つの領域を統合
    hsv_mask = maskHSV_low | maskHSV_high#画像のマスク（合成）
    resultHSV = cv2.bitwise_and(img, img, mask = hsv_mask)
    #cv2.imshow("Result HSV", resultHSV)
    #cv2.imshow("Result mask", hsv_mask)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    ret, img_binary = cv2.threshold(hsv_mask, 250, 255, cv2.THRESH_BINARY)
    #cv2.imshow('a',img_binary)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    white=cv2.countNonZero(img_binary)#白領域の画素数算出
    whiteratio=white/(160*120)#白領域の画素/全画素
    print("赤", whiteratio*100,"%")
    m = cv2.moments(img_binary)#白領域のモーメント算出
    if whiteratio > 0:
        cog_x = int(m["m10"] / m["m00"])
        #cog_y = int(m["m01"] / m["m00"])
        #print(width/4)
        #print("重心",cog_x)
        diffarence=cog_x-80
        if diffarence<-20:#左のとき,disigion=1
            #diffarence=-1*diffarence
            disigion=1
            print("zure",diffarence,"方向：","左")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'データ'+','+"赤コーン_重心位置"+","+"左"+'\n')
                file.write(makecontent()+','+'データ'+','+"赤コーン_重心位置"+","+str(diffarence)+"画素"+'\n')
        elif -20<diffarence<20:#真ん中のとき、disigion=0
            #diffarence=0
            disigion=0
            print("zure",diffarence,"方向：","中央")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'データ'+','+"赤コーン_重心位置"+","+"中央"+'\n')
                file.write(makecontent()+','+'データ'+','+"赤コーン_重心位置"+","+str(diffarence)+"画素"+'\n')
        else:#右の時、disigion=2
            disigion=2
            print("zure",diffarence,"方向：","右")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'データ'+','+"赤コーン_重心位置"+","+"右"+'\n')
                file.write(makecontent()+','+'データ'+','+"赤コーン_重心位置"+","+str(diffarence)+"画素"+'\n')
    else:
        disigion = 3
        print("akazero")
    return(diffarence,disigion,whiteratio)

def photo_process():
    goal=0
    while True:
        photo_result=gazousyori()
        ratio = photo_result[2]*100
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'データ'+','+"赤コーン_割合"+","+str(ratio)+"%"+'\n')
        print("ratio",ratio)
        if ratio>60:
            stop()
            stack()
            print("go-ru")
            goal=1
            break
        else:
            a=gazousyori()#重心のずれを測定
            diffarence=photo_result[0]
            disigion=photo_result[1]
            if disigion==1:#左のとき
                backspin_L(abs(diffarence)/250)
                print("hidarikaitenn")

            if disigion==2:
                backspin_R(abs(diffarence)/250)
                print("migikaitenn")

            if disigion==3:
                backspin_L(0.5)
                stop()
                stack()

            else:
                forward(4)
                print("zennshinn")

            stop()
            stack()
    return(goal)

def makecontent():
    dt=datetime.datetime.now()
    return str(dt)

if __name__=="__main__":
    with open("log.csv","w",encoding='utf-8')as file:
        file.write(makecontent()+','+'シーケンス'+','+str("log_start")+'\n')
    bmx_setup()
    motasetup()
    #housyutsuhanntei
    fall=0
    cyakuchi=0
    tegusu=0
    para=0
    yuudou=0
    ironinnshiki=0
    
    while True:
        fall=housyutsu2()
        if fall==1:
            print("housyutsukanryou")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'シーケンス'+','+"放出完了"+'\n')
            break
        else:
            print("tutunonaka")
    
    #cyakuchihanntei
    while True:
        cyakuchi=cyakuchi_hantei()
        if cyakuchi==1:
            print("cyakuchikannryou")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'シーケンス'+','+"着地完了"+'\n')
            break
        else:
            print("kuucyuu")
    
    #youdannkairo        
    tegusu=youdann()
    if tegusu==1:
        print("youdannkannryou")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'シーケンス'+','+"溶断完了"+'\n')
    else:
        print("youdann_error")
    
    #parakaihi
    para=parakaihi()
    if para==1:
        print("kaihi_kannryou")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'シーケンス'+','+"パラシュート回避完了"+'\n')
    else:
        print("kaihi_error")
    
    #GPSyuudou
    yuudou=GPS_yuudou()
    if yuudou==1:
        print("5m以内")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'シーケンス'+','+"GPS誘導完了"+'\n')
    else:
        print("error_yuudou")
    #gazousyori
    ironinnshiki=photo_process()
    if ironinnshiki==1:
        print("goal")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'シーケンス'+','+"GOAL!"+'\n')
    else:
        print("error_goal")
    
    
    
