import RPi.GPIO as GPIO
import time

class HallSensors:
    def __init__(self):
        self.Sensor_hall_motor = 11
        self.Sensor_hall_roda_A = 5
        self.Sensor_hall_roda_B = 6

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.Sensor_hall_motor, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.Sensor_hall_roda_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.Sensor_hall_roda_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.engine_pulse_count = 0
        self.last_engine_time = time.time()

        self.last_wheel_time = time.time()
        self.wheel_direction = 1 # Carro indo pra frente
        self.wheel_pulse_count = 0

        self.wheel_diameter = 0.63
        self.wheel_circumference = self.wheel_diameter * 3.1416

        GPIO.add_event_detect(self.Sensor_hall_motor, GPIO.RISING, callback=self.calc_engine_pulse, bouncetime=50)
        GPIO.add_event_detect(self.Sensor_hall_roda_A, GPIO.RISING, callback=self.calc_wheel_pulse, bouncetime=50)

    def calc_engine_pulse(self, channel):
        self.engine_pulse_count += 1

    def calc_wheel_pulse(self, channel):
        self.wheel_pulse_count += 1
        a_state = GPIO.input(self.Sensor_hall_roda_A)
        b_state = GPIO.input(self.Sensor_hall_roda_B)
        if a_state == GPIO.HIGH and b_state == GPIO.LOW:
            self.wheel_direction = 1
        elif a_state == GPIO.LOW and b_state == GPIO.HIGH:
            self.wheel_direction = -1

    def get_engine_rpm(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_engine_time
        self.last_engine_time = current_time

        rpm = (self.engine_pulse_count / elapsed_time) * 60
        self.engine_pulse_count = 0
        return rpm

    def get_wheel_speed(self):
        current_time = time.time()
        time_elapsed = current_time - self.last_wheel_time

        if time_elapsed > 0:  # Evitar divisão por zero
            # Velocidade linear = (Pulsos * Circunferência da Roda) / (Tempo decorrido)
            speed = (self.wheel_pulse_count * self.wheel_circumference) / time_elapsed
            speed *= self.wheel_direction  # Considerar direção do movimento
        else:
            speed = 0.0

        # Reset para a próxima medição
        self.wheel_pulse_count = 0
        self.last_wheel_time = current_time

        print(f"Wheel Pulse Count: {self.wheel_pulse_count}, Direction: {self.wheel_direction}")

        return speed * 3.6  # Converter m/s para km/h

    def stop(self):
        GPIO.remove_event_detect(self.Sensor_hall_motor)
        GPIO.remove_event_detect(self.Sensor_hall_roda_A)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()