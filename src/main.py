from src.modules.engine import Engine
from src.modules.pid import PID
from src.modules.hall_sensors import HallSensors
from src.modules.pedals import Pedals
import time
import threading

# Inicializando modulos
engine = Engine()
pid = PID()
pedals = Pedals()
hall_sensors = HallSensors()

# Tempo de loop
sampling_period = 0.2
running = True
current_speed = 0
timer = None

def main():
    try:
        # Executa a calibração e verifica se foi bem-sucedida
        if not calibrate_system():
            print("Calibração falhou. Encerrando o sistema...")
            close()
            return

        # Inicia a rotina principal
        routine()

        while running:
            time.sleep(1)
    except Exception as e:
        print(f"Erro: {e}")
        close()
    except KeyboardInterrupt:
        close()


def routine():
    global current_speed, timer

    if not running:
        return

    # Get reference speed from pedals
    #reference_speed = pedals.calculate_reference_speed(current_speed=current_speed)

    # Update the PID controller with the new reference speed
    #pid.refresh_reference(reference_speed)

    # Read the current wheel speed from the sensors
    measured_speed = hall_sensors.get_wheel_speed()

    # Calculate the control signal using PID
    #control_signal = pid.controller(measured_speed)
    control_signal = 80
    # Apply the control signal to the engine
    engine.moveEngine(control_signal)

    # Log the current state for debugging
    print(f"Measured Speed: {measured_speed:.2f} km/h, Current Speed: {current_speed:.2f} km/h")
    print(f"Control Signal: {control_signal}")

    #current_speed = measured_speed
    #current_speed += (measured_speed - current_speed) * 0.1

    # Re-agendar a execução do controle após o tempo de amostragem
    if running:
        timer = threading.Timer(sampling_period, routine)
        timer.start()

def calibrate_system():
    print("Iniciando calibração do sistema...")
    success = True

    try:
        # Teste do motor
        print("Testando Sensor_hall_motor...")
        engine.moveEngine(50)
        time.sleep(2)
        motor_rpm = hall_sensors.get_engine_rpm()
        engine.moveEngine(0)
        if motor_rpm > 0:
            print(f"Sensor_hall_motor funcionando: RPM = {motor_rpm:.2f}")
        else:
            print("Erro: Nenhum sinal no Sensor_hall_motor!")
            success = False

        # Teste das rodas
        print("Testando sensores de roda...")
        engine.moveEngine(30)
        time.sleep(3)
        wheel_speed = hall_sensors.get_wheel_speed()
        engine.moveEngine(0)
        if wheel_speed != 0:
            print(f"Sensores de roda funcionando: Velocidade = {wheel_speed:.2f} km/h")
        else:
            print("Erro: Nenhum sinal dos sensores de roda!")
            success = False

        # Verificação dos pedais
        print("Testando pedais...")
        if not (pedals.read_accelerator() or pedals.read_brake()):
            print("Erro: Nenhum sinal dos pedais!")
            success = False
        else:
            print("Pedais funcionando corretamente.")

    except Exception as e:
        print(f"Erro durante a calibração: {e}")
        success = False

    print("Calibração concluída.")
    return success


def close():
    global running, timer
    running = False

    if timer:
        timer.cancel()

    # Desligar componentes
    engine.moveEngine(0)
    hall_sensors.stop()
    pedals.stop()
    time.sleep(1)
    print('Sistema encerrado')

if __name__ == '__main__':
    main()