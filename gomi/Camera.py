from fpioa_manager import fm
from machine import UART
import _thread
import time,sys
from modules import ws2812

# Initialize of UART
fm.register(35, fm.fpioa.UART2_TX, force=True)
fm.register(34, fm.fpioa.UART2_RX, force=True)
uart_Port = UART(UART.UART2, 57600, 8, None, 1, timeout=1000, read_buf_len= 4096)

class_ws2812 = ws2812(8, 1)
a = class_ws2812.set_led(0, (100, 0, 100))
a = class_ws2812.display()
time.sleep(1)
a = class_ws2812.set_led(0, (0, 0, 0))
a = class_ws2812.display()

def sendData(a, uart_Port):
    num = 1
    while True:
        byte_size = uart_Port.write("From UnitV:" + str(num) + "\n")
        print("Send:" + str(num))
        num = num + 1
        a = class_ws2812.set_led(0, (10 , 0, 0))
        a = class_ws2812.display()
        time.sleep(0.1)
        a = class_ws2812.set_led(0, (0, 0, 0))
        a = class_ws2812.display()
        time.sleep_ms(1000)

def recvData(a, uart_Port):
    while True:
        if uart_Port.any():
            recv_data = uart_Port.readline()
            recv_str = recv_data.decode('utf-8')
            print("Recv:" + recv_str)
            time.sleep(0.1)
            a = class_ws2812.set_led(0, (0 , 10, 0))
            a = class_ws2812.display()
            time.sleep(0.1)
            a = class_ws2812.set_led(0, (0, 0, 0))
            a = class_ws2812.display()
    time.sleep_ms(1)
a = "test"
_thread.start_new_thread(sendData, (a, uart_Port))
_thread.start_new_thread(recvData, (a, uart_Port))

try:
    while True:
        time.sleep(1)
except Exception as e:
    sys.print_exception(e)
    sys.exit()
