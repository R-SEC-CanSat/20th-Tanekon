import serial
import time
def GPS():
    # シリアル通信設定
    uart = serial.Serial('/dev/serial0', 9600, timeout = 10)
    # gps設定

    # 10秒ごとに表示
    tm_last = 0
    while True:  　　　　　　
        sentence = uart.readline()
        print(sentence)
        time.sleep(1)
        
if __name__=="__main__":
    GPS()
    