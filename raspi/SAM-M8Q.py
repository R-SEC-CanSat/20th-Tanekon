from smbus2 import SMBus
import time

# I2C communication parameters
DEFAULT_DEVICE_ADDRESS = 0x42
I2C_DELAY = 0.001
i2c = SMBus(1)

# I2C read data structures
buff = [0] * 80
idx = 0
lat = [''] * 9
lon = [''] * 10

# Read 80 bytes from I2C
def readI2C():
    block = i2c.read_i2c_block_data(DEFAULT_DEVICE_ADDRESS, 0, 32)
    print(block)



def main():
    while True:
        readI2C()
    

if __name__ == "__main__":
    main()