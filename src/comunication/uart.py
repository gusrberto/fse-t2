import serial
import struct
from .crc16 import *
import time
import threading

mat_digits = bytes([9, 4, 5, 7])

address_table = {
    'seta': 0x00,
    'cruise_control': 0x01,
    'farol': 0x02,
    'velocidade': 0x03,
    'rotacao_motor': 0x07,
    'seta_esq': 0x0B,
    'seta_dir': 0x0C,
    'farol_alto': 0x0D,
    'farol_baixo': 0x0E
}

class Uart:
    def __init__(self, lock=None):
        self.serial = serial.Serial(port="/dev/serial0",
                                    baudrate=115200,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS,
                                    timeout=0.5)
        if self.serial is not None: print("UART Inicializada")
        
        self.uart_lock = lock
        self.seta_esq_active = False
        self.seta_dir_active = False
        self.seta_esq_thread = None
        self.seta_dir_thread = None

    def connect(self) -> None:
        if self.serial is not None and not self.serial.is_open:
            self.serial.open()
            print("UART conectada com sucesso!")

    def disconnect(self) -> None:
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            print("UART desconectada com sucesso!")

    def crc_validate(self, buffer, buffer_size):
        crc_size = 2
        if len(buffer) < crc_size:
            return False
        crc_buffer = buffer[-crc_size:]
        crc = calculate_crc(buffer[:-crc_size], buffer_size - crc_size).to_bytes(crc_size, 'little')
        return crc_buffer == crc
    
    def get_address(self, information, bytes_size):
        address = address_table.get(information)
        if address is None:
            raise ValueError(f"Endereço inválido para a informação: {information}")
        return bytes([address, bytes_size])
    
    def send_message(self, message, message_type):
        response = None
        with self.uart_lock:
            self.serial.write(message)
            time.sleep(0.05)

            if (message_type == "ler_temperatura"):
                response = self.serial.read(9)
            elif (message_type == "escrever_byte_registrador"):
                response = self.serial.read(5)
            elif (message_type == "escrever_float_registrador"):
                response = self.serial.read(8)
            elif (message_type == "ler_registrador"):
                response = self.serial.read(8)
        
        return response
    
    def read_temp_value(self):
        with self.uart_lock: 
            temp = 100 #fake value
            round_temp = round(temp, 2)

            message = bytes([0x01, 0x23, 0xAA]) + mat_digits
            print(message)
            crc = calculate_crc(message, len(message))
            message += crc

            response = self.send_message(message, "ler_temperatura")

            crc_status = self.crc_validate(response, len(response))

            if crc_status:
                temp_value = struct.unpack(">f", response[3:7])[0]
                print(f"Temperatura do motor lida: {temp_value}")
                return temp_value
            else:
                print("Erro no CRC (Leitura de valor de temperatura do motor)")

    def read_registers_byte(self, information):
        with self.uart_lock: 
            message = bytes([0x01, 0x03]) + self.get_address(information, 1) + mat_digits
            print(message)
            crc = calculate_crc(message, len(message))
            message += crc

            response = self.send_message(message, "ler_registrador")
            crc_status = self.crc_validate(response, len(response))

            if crc_status:
                register_value = int.from_bytes(response[2:3], byteorder='big', signed=False)
                return register_value
            else:
                return print("Erro no CRC (Leitura registradores Float)")
            
    def read_registers_float(self, information):
        with self.uart_lock: 
            message = bytes([0x01, 0x03]) + self.get_address(information, 4) + mat_digits
            print(message)
            crc = calculate_crc(message, len(message))
            message += crc

            response = self.send_message(message, "ler_registrador")
            crc_status = self.crc_validate(response, len(response))

            if crc_status:
                float_bytes = response[3:7]

                float_value = struct.unpack('>f', float_bytes)[0]
                return float_value
            else:
                return print("Erro no CRC (Leitura registradores Float)")
        
    def write_registers_byte(self, information, data):
        with self.uart_lock: 
            message = bytes([0x01, 0x06]) + self.get_address(information, 1) + bytes([data]) + mat_digits
            print(message)
            crc = calculate_crc(message, len(message))
            message += crc

            response = self.send_message(message, "escrever_byte_registrador")

            crc_status = self.crc_validate(response, len(response))

            if crc_status:
                register_value = int.from_bytes(response[2:3], byteorder='big', signed=False)
                print(f"Valor do registrador (byte): {register_value}")
                return register_value
            else:
                return print("Erro no CRC (Escrita registrador byte)")
        
    def write_registers_float(self, information, data):
        with self.uart_lock: 
            float_bytes = struct.pack('<f', data)
            message = bytes([0x01, 0x06]) + self.get_address(information, 4) + float_bytes + mat_digits
            print(message)
            crc = calculate_crc(message, len(message))
            message += crc

            response = self.send_message(message, "escrever_float_registrador")

            crc_status = self.crc_validate(response, len(response))

            if crc_status:
                float_bytes_response = response[3:7]

                float_value_response = struct.unpack('>f', float_bytes_response)
                return float_value_response
            else:
                return print("Erro no CRC (Escrita registrador float)")

    def blink_signal_pannel(self):
        while (self.seta_esq_active) or (self.seta_dir_active):
            with self.uart_lock:
                if self.seta_esq_active:
                    self.write_registers_byte("seta_esq", 1)
                    time.sleep(0.5)
                    self.write_registers_byte("seta_esq", 0)
                    time.sleep(0.5)
                elif self.seta_dir_active:
                    self.write_registers_byte("seta_dir", 1)
                    time.sleep(0.5)
                    self.write_registers_byte("seta_dir", 0)
                    time.sleep(0.5)

    def left_signal_blink_pannel(self):
        if not self.seta_esq_active:
            self.seta_esq_active = True
            self.seta_esq_thread = threading.Thread(target=self.blink_signal_pannel)
            self.seta_esq_thread.start()

    def left_signal_off_panel(self):
        if self.seta_esq_active:
            self.seta_esq_active = False
            self.seta_esq_thread.join()
            self.seta_esq_thread = None
            self.write_registers_byte("seta_esq", 0)

    def right_signal_blink_pannel(self):
        if not self.seta_dir_active:
            self.seta_dir_active = True
            self.seta_dir_thread = threading.Thread(target=self.blink_signal_pannel)
            self.seta_dir_thread.start()

    def right_signal_off_panel(self):
        if self.seta_dir_active:
            self.seta_dir_active = False
            self.seta_dir_thread.join()
            self.seta_dir_thread = None
            self.write_registers_byte("seta_dir", 0)           