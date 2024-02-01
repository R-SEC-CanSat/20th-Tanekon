from fpioa_manager import fm
from machine import UART
import time
import sensor
import image

fm.register(10, fm.fpioa.UART1_TX, force=True)
fm.register(11, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)
sensor.reset()                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565) # RGB565フォーマットを使用
sensor.set_framesize(sensor.QVGA)   # QVGAフレームサイズを使用
sensor.set_auto_gain(False)         # オートゲインをオフ
sensor.__write_reg(0x80, 0xEF)      # オートホワイトバランスをオフ、白飛び防止
sensor.skip_frames(time = 2000)     # 2秒間フレームをスキップして安定化
threshold = [(30, 100,  15, 127,  15, 127)] # 検出する色のしきい値を設定
sensor.run(1)# 連続取得開始

Px = -0.4

while True:
    img = sensor.snapshot()
    #連続ピクセルの塊の位置と大きさを取得
    blobs = img.find_blobs(threshold, x_stride = 2, y_stride = 2, pixels_threshold = 100, merge = True, margin = 20)
    if blobs:
        max_area = 0
        target = blobs[0]
        for b in blobs:
            if b.area() > max_area:
                max_area = b.area()
                target = b

        xerr = 160 - target[5]
        ux = int(Px * xerr)

        if ux > 100:
            ux = 100
        elif ux < -100:
            ux = -100

        if abs(xerr) < 20:
            ux = 0
        #print(ux)

        if uart.read(4096):
            area = target.area()
            hexlist = [ux & 0xFF, (area >> 16) & 0xFF, (area >> 8) & 0xFF, area & 0xFF]
            a = uart.write(bytes(hexlist))

        #print(abs(prev_area - target.area()))
        prev_area = target.area()
        tmp=img.draw_rectangle(target[0:4])
        tmp=img.draw_cross(target[5], target[6])
        # c=img.get_pixel(target[5], target[6])
    else:
        if uart.read(1):
            hexlist = [0x00, 0x00, 0x00, 0x00]
            a = uart.write(bytes(hexlist))