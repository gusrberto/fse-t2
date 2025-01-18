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

def get_code(code, value=0, command=None, quantity=1):
    if code == 'solicita_temp_motor':
        return bytes([0x01, 0x23, 0xAA]) + mat_digits