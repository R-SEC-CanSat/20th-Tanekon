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
sensor.set_brightness(-3)            # 明度の調整-3から3
sensor.set_saturation(0)            # 彩度の調整
sensor.set_contrast(2)              # コントラストの調整

#red
threshold = [(27, 100, 23, 127, -17, 41),
             ((14, 90, 15, 127, 27, 57)),
             (64, 95, -30, 61, 27, 127)]

while(True):
    #red_detect
    img = sensor.snapshot()           # 画像を取得
    blobs = img.find_blobs(threshold) # しきい値内の色を検出
    if blobs:
        if b.code() == 1:   # Red
            max_blob = max(blobs, key=lambda b: b.area()) # 面積が最大の領域を取得
            img.draw_rectangle(max_blob[0:4])             # 検出した色を矩形で囲む
            img.draw_cross(max_blob[5], max_blob[6])      # 検出した色の中心に十字を描く
            max_per_red = max_blob.area()/307200
            red_x = max_blob[5]
            red_y = max_blob[6]
            red_area = max_per_red
            print("R",max_blob[5], max_blob[6],max_per_red)

        if b.code() == 2:   # Green
            max_blob = max(blobs, key=lambda b: b.area()) # 面積が最大の領域を取得
            img.draw_rectangle(max_blob[0:4])             # 検出した色を矩形で囲む
            img.draw_cross(max_blob[5], max_blob[6])      # 検出した色の中心に十字を描く
            max_per_ore = max_blob.area()/307200
            ore_x = max_blob[5]
            ore_y = max_blob[6]
            ore_area = max_per_ore
            print("O",max_blob[5], max_blob[6],max_per_ore)
        if b.code() == 4:   # Blue
            max_blob = max(blobs, key=lambda b: b.area()) # 面積が最大の領域を取得
            img.draw_rectangle(max_blob[0:4])             # 検出した色を矩形で囲む
            img.draw_cross(max_blob[5], max_blob[6])      # 検出した色の中心に十字を描く
            max_per_ye = max_blob.area()/307200
            ye_x = max_blob[5]
            ye_y = max_blob[6]
            ye_area = max_per_ye
            print("Y",max_blob[5], max_blob[6],max_per_ye)
        #sendstr = "R" + str(max_blob_red[5])+","+str(max_blob_red[6])+","+str(max_per_red)
        #for char in sendstr:
        #    uart.write(char)
        #uart.write("\n")
        #print(max_blob_red[5], max_blob_red[6],max_per_red)

    else:
        sendstr = "R0,0,0.0O0,0,0.0Y0,0,0.0"
        for char in sendstr:
            uart.write(char)
        uart.write("\n")
        print("0,0,0")
    time.sleep(0.001)


