import sensor
import image
from fpioa_manager import fm
from machine import UART
from Maix import GPIO
from modules import ws2812
import time

# RGB LED設定
class_ws2812 = ws2812(8,100)
BRIGHTNESS = 0x10

fm.register(34, fm.fpioa.UART2_TX, force=True)
fm.register(35, fm.fpioa.UART2_RX, force=True)
uart = UART(UART.UART2, 57600, 8, None, 1, timeout=1000, read_buf_len= 4096)

def RGB_LED(r,g,b):
    a = class_ws2812.set_led(0,(r,g,b))
    a = class_ws2812.display()
    time.sleep(0.3)
    a = class_ws2812.set_led(0,(0,0,0))
    a = class_ws2812.display()
    time.sleep(0.3)
    a = class_ws2812.set_led(0,(r,g,b))
    a = class_ws2812.display()

# 起動インジケータとしてblueで点滅、最後に点灯
RGB_LED(0,0,BRIGHTNESS)
RGB_LED(0,0,BRIGHTNESS)
RGB_LED(0,0,BRIGHTNESS)

sensor.reset()                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565) # RGB565フォーマットを使用
sensor.set_framesize(sensor.VGA)   # VGAフレームサイズを使用
#sensor.set_auto_gain(False)         # オートゲインをオフ
sensor.__write_reg(0x80, 0xEF)      # オートホワイトバランスをオフ
sensor.skip_frames(time = 2000)     # 2秒間フレームをスキップして安定化
sensor.set_brightness(0)            # 明度の調整
sensor.set_saturation(0)            # 彩度の調整
sensor.set_contrast(2)              # コントラストの調整


#red
threshold_red = [(27, 100, 23, 127, -17, 41)] # 検出する色のしきい値を設定
#orange
threshold_orange = [(41, 95, -2, 37, -20, 41)] # 検出する色のしきい値を設定
#yellow
threshold_yellow = [(27, 100, 23, 127, -17, 41)] # 検出する色のしきい値を設定

while(True):
    #red_detect
    img = sensor.snapshot()           # 画像を取得
    #赤色を検出した場合
    blobs_red = img.find_blobs(threshold_red) # しきい値内の色を検出
    if blobs_red :
        max_blob_red = max(blobs_red, key=lambda b: b.area()) # 面積が最大の領域を取得
        max_per_red = max_blob_red.area()/307200
        img.draw_rectangle(max_blob_red[0:4])             # 検出した色を矩形で囲む
        img.draw_cross(max_blob_red[5], max_blob_red[6])# 検出した色の中心に十字を描く
        sendstr = "R" + str(max_blob_red[5])+","+str(max_blob_red[6])+","+str(max_per_red)
        print(max_blob_red[5], max_blob_red[6],max_per_red)
    else:
        sendstr = "R0,0,0.0"
        print("R0,0,0")
    #オレンジ色を検出した場合
    blobs_orange = img.find_blobs(threshold_orange) # しきい値内の色を検出
    if blobs_orange :
        max_blob_orange = max(blobs_orange, key=lambda b: b.area()) # 面積が最大の領域を取得
        max_per_orange = max_blob_orange.area()/307200
        img.draw_rectangle(max_blob_orange[0:4])             # 検出した色を矩形で囲む
        img.draw_cross(max_blob_orange[5], max_blob_orange[6])
        sendstr += "O" + str(max_blob_orange[5])+","+str(max_blob_orange[6])+","+str(max_per_orange)
        print(max_blob_orange[5], max_blob_orange[6],max_per_orange)
    else:
        sendstr += "O0,0,0.0"
        print("0,0,0")
    #yellow_detect
    blobs_yellow = img.find_blobs(threshold_yellow) # しきい値内の色を検出
    if blobs_yellow :
        max_blob_yellow = max(blobs_yellow, key=lambda b: b.area()) # 面積が最大の領域を取得
        max_per_yellow = max_blob_yellow.area()/307200
        img.draw_rectangle(max_blob_yellow[0:4])             # 検出した色を矩形で囲む
        img.draw_cross(max_blob_yellow[5], max_blob_yellow[6])
        sendstr += "Y" + str(max_blob_yellow[5])+","+str(max_blob_yellow[6])+","+str(max_per_yellow)
        print(max_blob_yellow[5], max_blob_yellow[6],max_per_yellow)
    else:
        sendstr += "Y0,0,0.0"
        print("0,0,0")
    for char in sendstr:
        uart.write(char)
    uart.write("\n")
    time.sleep(0.001)


