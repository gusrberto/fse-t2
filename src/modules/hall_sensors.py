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

        self.pos_i = 0
        self.previous_time = 0
        self.pos_prev = 0

        self.wheel_diameter = 0.63
        self.wheel_circumference = self.wheel_diameter * 3.1416

        GPIO.add_event_detect(self.Sensor_hall_motor, GPIO.RISING, callback=self.calc_engine_pulse, bouncetime=1)
        GPIO.add_event_detect(self.Sensor_hall_roda_A, GPIO.RISING, callback=self.readEncoder, bouncetime=1)

    def readEncoder(self, channel):
        b_state = GPIO.input(self.Sensor_hall_roda_B)
        increment = 0
        if b_state > 0:
            increment = 1
        else:
            increment = -1
        self.pos_i = self.pos_i + increment

    def compute_velocity(self):
        pos = 0
        pos = self.pos_i

        current_time = time.time()
        elapsed_time = (current_time - self.previous_time) / 1.0e6
        velocity1 = (pos - self.pos_prev) / elapsed_time
        self.pos_prev = pos
        self.previous_time = current_time

        print(f"Velocidade 1: {velocity1}")

        return velocity1


    def calc_engine_pulse(self, channel):
        self.engine_pulse_count += 1

    def calc_wheel_pulse(self, channel):
        self.wheel_pulse_count += 1
        a_state = GPIO.input(self.Sensor_hall_roda_A)
        b_state = GPIO.input(self.Sensor_hall_roda_B)
        print(f"A state: {a_state}, B state: {b_state}")
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
        elapsed_time = current_time - self.last_wheel_time

        if elapsed_time > 0:
            # Calcula a velocidade com base nos pulsos e no tempo decorrido
            wheel_speed = (self.wheel_pulse_count * self.wheel_circumference / elapsed_time) * 3.6 * self.wheel_direction
        else:
            wheel_speed = 0.0

        # Adiciona depuração para verificar os valores ANTES de resetar
        print(f"Pulse Count: {self.wheel_pulse_count}, Elapsed Time: {elapsed_time:.2f}, Speed: {wheel_speed:.2f} km/h")
        # Reseta o contador de pulsos para a próxima leitura
        self.wheel_pulse_count = 0
        self.last_wheel_time = current_time

        return wheel_speed

    def stop(self):
        GPIO.remove_event_detect(self.Sensor_hall_motor)
        GPIO.remove_event_detect(self.Sensor_hall_roda_A)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()