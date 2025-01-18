import serial

from .crc16 import *

class Uart:
    def __init__(self):
        self.serial = serial.Serial(port="/dev/serial0",
                                    baudrate=115200,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS,
                                    timeout=0.5)
        if self.serial is not None: print("UART Inicializada")

    def connect(self) -> None:
        if self.serial is not None and not self.serial.is_open:
            self.serial.open()

    def disconnect(self) -> None:
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

    def crc_validate(self, buffer, buffer_size):
        crc_size = 2
        if len(buffer) < crc_size:
            return False
        crc_buffer = buffer[-crc_size:]
        crc = calculate_crc(buffer[:-crc_size], buffer_size - crc_size).to_bytes(crc_size, 'little')
        return crc_buffer == crc
    
"""     def read(self, ):
        try:
            self.connect() """