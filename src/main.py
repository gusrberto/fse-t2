from src.modules.pid import PID
from src.modules.hall_sensors import HallSensors
from src.modules.vehicle_control import VehicleControl

from src.comunication.uart import Uart

import RPi.GPIO as GPIO
import time
import threading

# Inicializando modulos
engine_pid = PID()
hall_sensors = HallSensors()
vehicle_control = VehicleControl(max_speed=200)

uart_lock = threading.Lock()
uart = Uart(lock=uart_lock)

# Tempo de loop
sampling_period = 0.05

# Variáveis globais
running = True
current_speed = 0
engine_rpm = 0
timer = None
routine_thread = None
uart_thread = None

def main():
    global routine_thread, uart_thread
    try:
        # Inicia a comunicação UART
        uart.connect()

        # Executa a calibração e verifica se foi bem-sucedida
        if not calibrate_system(vehicle_control, hall_sensors):
            print("Calibração falhou. Encerrando o sistema...")
            close()
            return

        # Inicia a rotina principal de controle do veículo
        routine_thread = threading.Thread(target=routine)
        routine_thread.start()

        # Inicia a rotina de comunicação com a UART
        uart_thread = threading.Thread(target=uart_listener)
        uart_thread.start()

        while running:
            time.sleep(1)
    except Exception as e:
        print(f"Erro: {e}")
        close()
    except KeyboardInterrupt:
        close()


def routine():
    global current_speed, engine_rpm

    while running:
        pedal_ac = vehicle_control.read_accelerator_pedal()
        pedal_fr = vehicle_control.read_brake_pedal()

        print(f"Acelerador: {pedal_ac} | Freio: {pedal_fr}")

        target_speed = vehicle_control.calculate_target_speed(pedal_ac, pedal_fr)

        engine_pid.refresh_reference(target_speed)

        measured_speed = hall_sensors.compute_wheel_velocity()

        pid_control_signal = engine_pid.controller(measured_speed)

        vehicle_control.engine_controller(pid_control_signal)

        current_speed = measured_speed
        engine_rpm = hall_sensors.get_engine_rpm()

        # 8. Log para depuração
        print(f"Target Speed: {target_speed:.2f} km/h, Measured Speed: {measured_speed:.2f} km/h, PID Control Signal: {pid_control_signal:.2f}")
        print(f"Engine RPM: {hall_sensors.get_engine_rpm()}")
        print(f"Current Speed: {current_speed}")

        time.sleep(sampling_period)

def calibrate_system(vehicle_control, hall_sensors):
    print("Iniciando calibração do sistema...")
    success = True

    try:
        # Teste do motor
        print("Testando Sensor_hall_motor...")
        vehicle_control.set_engine_mode(GPIO.HIGH, GPIO.LOW, 50, 0)  # Motor a 50% para frente
        time.sleep(2)
        motor_rpm = hall_sensors.get_engine_rpm()
        vehicle_control.set_engine_mode(GPIO.LOW, GPIO.LOW, 0, 0)  # Desliga o motor

        if motor_rpm > 0:
            print(f"Sensor_hall_motor funcionando: RPM = {motor_rpm:.2f}")
        else:
            print("Erro: Nenhum sinal no Sensor_hall_motor!")
            success = False

        # Teste das rodas
        print("Testando sensores de roda...")
        vehicle_control.set_engine_mode(GPIO.HIGH, GPIO.LOW, 30, 0)  # Motor a 30% para frente
        time.sleep(3)
        wheel_speed = hall_sensors.compute_wheel_velocity()
        vehicle_control.set_engine_mode(GPIO.LOW, GPIO.LOW, 0, 0)  # Desliga o motor

        if wheel_speed != 0:
            print(f"Sensores de roda funcionando: Velocidade = {wheel_speed:.2f} km/h")
        else:
            print("Erro: Nenhum sinal dos sensores de roda!")
            success = False

    except Exception as e:
        print(f"Erro durante a calibração: {e}")
        success = False

    print("Calibração concluída.")
    return success

def uart_listener():
    signal_previous_state = None
    lights_previous_state = None

    while running:
        try:
            # Controle Farol Alto e Baixo
            lights_button = uart.read_registers_byte("farol")
            print(f"Valor do botao do farol: {lights_button}")

            if lights_button != lights_previous_state:
                if lights_button == 1:
                    low_lights_value = uart.read_registers_byte("farol_baixo")
                    uart.write_registers_byte("farol_alto", 0)
                    uart.write_registers_byte("farol_baixo", int(not low_lights_value))
                    vehicle_control.control_lights(farol_baixo=True, farol_alto=False)
                elif lights_button == 2:
                    high_lights_value = uart.read_registers_byte("farol_alto")
                    uart.write_registers_byte("farol_alto", int(not high_lights_value))
                    uart.write_registers_byte("farol_baixo", 0)
                    vehicle_control.control_lights(farol_baixo=False, farol_alto=True)

                lights_previous_state = lights_button
                uart.write_registers_byte("farol", 0)

            # Controle setas esquerda e direita
            signal_button = uart.read_registers_byte("seta")
            print(f"Valor do botao da seta: {signal_button}")

            if signal_button != signal_previous_state:
                if signal_button == 1:
                    left_signal_value = uart.read_registers_byte("seta_esq")
                    vehicle_control.right_signal_off()
                    uart.right_signal_off_panel()
                    if (left_signal_value == 0):
                        vehicle_control.left_signal_blink()
                        uart.left_signal_blink_pannel()
                    elif (left_signal_value == 1):
                        vehicle_control.left_signal_off()
                        uart.left_signal_off_panel()
                elif signal_button == 2:
                    right_signal_value = uart.read_registers_byte("seta_dir")
                    vehicle_control.left_signal_off()
                    uart.left_signal_off_panel()
                    if (right_signal_value == 0):
                        vehicle_control.right_signal_blink()
                        uart.right_signal_blink_pannel()
                    elif (right_signal_value == 1):
                        vehicle_control.left_signal_off()
                        uart.left_signal_off_panel()

                signal_previous_state = signal_button
                uart.write_registers_byte("seta", 0)

            # Escrevendo velocidade e rpm no painel
            #uart.write_registers_float("velocidade", current_speed)
            #uart.write_registers_float("rotacao_motor", engine_rpm)

        except Exception as e:
            print(f"Erro na função UART Listener: {e}")
        time.sleep(0.05) # 50ms

def close():
    global running, routine_thread, uart_thread
    running = False

    if routine_thread is not None:
        routine_thread.join()

    if uart_thread is not None:
        uart_thread.join()

    # Desligar componentes
    uart.disconnect()
    hall_sensors.stop()
    vehicle_control.cleanup()
    time.sleep(1)
    print('Sistema encerrado')

if __name__ == '__main__':
    main()