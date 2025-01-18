from src.modules.engine import Engine
from src.modules.pid import PID
from src.modules.hall_sensors import HallSensors
from src.modules.pedals import Pedals
import time
import threading
import RPi.GPIO as GPIO 

GPIO.setmode(GPIO.BCM)

# Inicializando modulos
engine = Engine()
pid = PID()
pedals = Pedals()
hall_sensors = HallSensors()

encerrar = threading.Event()

# Tempo de loop
sampling_period = 0.5

running = True

current_speed = 0

def main():
    try:
        routine()  # Inicia a rotina principal
        while not encerrar.is_set():
            time.sleep(1)  # Reduz a ocupação da CPU
    except KeyboardInterrupt:
        print("Encerrando pelo usuário...")
    except Exception as e:
        print(f"Erro inesperado: {e}")
    finally:
        close()

def routine():
    global current_speed

    try:
        # Obter a velocidade de referência dos pedais
        reference_speed = pedals.calculate_reference_speed(current_speed=current_speed)

        # Atualizar o controlador PID com a nova velocidade de referência
        pid.refresh_reference(reference_speed)

        # Ler a velocidade atual da roda pelos sensores
        measured_speed = hall_sensors.get_wheel_speed()

        # Calcular o sinal de controle usando PID
        control_signal = pid.controller(measured_speed)

        # Aplicar o sinal de controle ao motor
        engine.moveEngine(control_signal)

        # Log para depuração
        print(f"Reference Speed: {reference_speed:.2f} km/h, Measured Speed: {measured_speed:.2f} km/h, Control Signal: {control_signal:.2f}")
        print(f"Current Speed: {current_speed:.2f} km/h")
        current_speed = measured_speed

    except Exception as e:
        print(f"Erro na rotina: {e}")
        close()
        return

    # Reagendar a execução do controle após o tempo de amostragem
    if not encerrar.is_set():
        threading.Timer(sampling_period, routine).start()


def close():
    encerrar.set()  # Sinaliza para parar o agendamento
    engine.moveEngine(0)
    hall_sensors.stop()
    pedals.stop()
    print('Sistema encerrado')

if __name__ == '__main__':
    main()