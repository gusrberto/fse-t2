from bmp280 import BMP280
import smbus2

class bmp280_device:
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self.bmp280 = BMP280(i2c_dev=self.bus)

    def get_temperature_bmp280(self):
        return round(self.bmp280.get_temperature(), 2)