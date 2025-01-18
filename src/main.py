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
    reference_speed = pedals.calculate_reference_speed(current_speed=current_speed)

    # Update the PID controller with the new reference speed
    pid.refresh_reference(reference_speed)

    # Read the current wheel speed from the sensors
    measured_speed = hall_sensors.get_wheel_speed()

    # Calculate the control signal using PID
    control_signal = pid.controller(measured_speed)

    # Apply the control signal to the engine
    engine.moveEngine(control_signal)

    # Log the current state for debugging
    print(f"Reference Speed: {reference_speed:.2f} km/h, Measured Speed: {measured_speed:.2f} km/h, Control Signal: {control_signal:.2f}")
    print(f"Current Speed: {current_speed:.2f} km/h")

    current_speed = measured_speed

    # Re-agendar a execução do controle após o tempo de amostragem
    if running:
        timer = threading.Timer(sampling_period, routine)
        timer.start()

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