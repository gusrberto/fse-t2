from src.modules.pid import PID
from src.modules.hall_sensors import HallSensors
from src.modules.vehicle_control import VehicleControl

from src.comunication.uart import Uart
from src.comunication.modbus import get_code

import RPi.GPIO as GPIO
import time
import threading

# Inicializando modulos
engine_pid = PID()
hall_sensors = HallSensors()
vehicle_control = VehicleControl(max_speed=200)
uart = Uart()

uart_lock = threading.Lock()

# Tempo de loop
sampling_period = 0.2
running = True
current_speed = 0
timer = None
routine_thread = None

def main():
    global routine_thread
    try:
        # Executa a calibração e verifica se foi bem-sucedida
        if not calibrate_system(vehicle_control, hall_sensors):
            print("Calibração falhou. Encerrando o sistema...")
            close()
            return

        # Inicia a rotina principal
        routine_thread = threading.Thread(target=routine)
        routine_thread.start()

        while running:
            time.sleep(1)
    except Exception as e:
        print(f"Erro: {e}")
        close()
    except KeyboardInterrupt:
        close()


def routine():
    global current_speed

    while running:
        vehicle_control.update_lights_state()

        pedal_ac = vehicle_control.read_accelerator_pedal()
        pedal_fr = vehicle_control.read_brake_pedal()

        print(f"Acelerador: {pedal_ac} | Freio: {pedal_fr}")

        target_speed = vehicle_control.calculate_target_speed(pedal_ac, pedal_fr)

        engine_pid.refresh_reference(target_speed)

        measured_speed = hall_sensors.compute_wheel_velocity()

        pid_control_signal = engine_pid.controller(measured_speed)

        vehicle_control.engine_controller(pid_control_signal)

        current_speed = measured_speed

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

def uart_listener(message, value=0, command=None):
    while running:
        try:
            if message == 'le_registrador':
                with uart_lock:
                    full_message = get_code(message)
                    uart.write(full_message, len(full_message))
                    response = uart.read(15)

                    if isinstance(response, str):
                        response = response.encode('utf-8')

                    return response
        except Exception as e:
            print(f"Erro na função UART Listener: {e}")
        time.sleep(0.05) # 50ms

def close():
    global running, routine_thread
    running = False

    if routine_thread is not None:
        routine_thread.join()

    # Desligar componentes
    hall_sensors.stop()
    vehicle_control.cleanup()
    time.sleep(1)
    print('Sistema encerrado')

if __name__ == '__main__':
    main()