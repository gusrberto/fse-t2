import struct

mat_digits = bytes([9, 4, 5, 7])

command_table = {
    'seta_esq_dir': 0x00,
    'cruise_control': 0x01,
    'farol': 0x02
}

display_table = {
    'velocidade': 0x03,
    'rotacao_motor': 0x07,
    'seta_esq': 0x0B,
    'seta_dir': 0x0C,
    'farol_alto': 0x0D,
    'farol_baixo': 0x0E
}

def get_code(code, value=0, command=None):
    if code == 'solicita_temp_motor':
        return bytes([0x01, 0x23, 0xAA]) + mat_digits
    elif code == 'le_registrador':
        return bytes([0x01, 0x03]) + get_address(command, 1, command_table) + mat_digits
    elif code == 'escreve_registrador':
        return bytes([0x01, 0x06]) + get_address(command, 1, command_table) + value.to_bytes(1, 'little') + mat_digits
    
def get_address(information, bytes_size, table):
    address = table.get(information)
    sub_code = bytes([address, bytes_size])
    return sub_code