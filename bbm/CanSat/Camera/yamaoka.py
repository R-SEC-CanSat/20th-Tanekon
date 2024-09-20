import sensor, image, time, utime
from machine import UART
from Maix import GPIO
from fpioa_manager import fm
from modules import ws2812

class_ws2812 = ws2812(8, 1)

fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)

fm.register(18, fm.fpioa.GPIO1)
ButonA = GPIO(GPIO.GPIO1, GPIO.IN, GPIO.PULL_UP)
fm.register(19, fm.fpioa.GPIO2)
ButonB = GPIO(GPIO.GPIO2, GPIO.IN, GPIO.PULL_UP)

uart_out = UART(UART.UART1, 115200,8,None,1, timeout=1000, read_buf_len=4096)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.run(1)

while False:
    uart_out.write('TEST/n')
    utime.sleep_ms(100)

target_lab_threshold = (30,100,15,127,15,127)
while True:
    img=sensor.snapshot()

    if ButonA.value() == 0:
        b = class_ws2812.set_led(0, (50, 0, 10))
        b = class_ws2812.display()
        time.sleep(0.5)

    if ButonB.value() == 0:
        b = class_ws2812.set_led(0, (0, 50, 50))
        b = class_ws2812.display()
        time.sleep(0.5)

    blobs = img.find_blobs([target_lab_threshold], x_stride = 2, y_stride = 2, pixels_threshold = 100, merge = False, margin = 20)
    if blobs:
        max_area = 0
        target = blobs[0]
        for b in blobs:
                if b.area() > max_area:
                    max_area = b.area()
                    target = b
        img.draw_rectangle(max_area[0:4])             # 検出した色を矩形で囲む
        img.draw_cross(max_area[5], max_area[6])
        if uart_out.read(4096):
            area = target.area()
            dx = 160 - target[5]
            hexlist = [(dx >> 8) & 0xFF, dx & 0xFF, (area >> 16) & 0xFF, (area >> 8) & 0xFF, area & 0xFF]
            uart_out.write(bytes(hexlist))
        else:
            pass
        print(target[5],target.area())