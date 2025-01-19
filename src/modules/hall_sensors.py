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
        self.previous_engine_time = time.time()

        self.wheel_pos_i = 0
        self.previous_wheel_time = time.time()
        self.pos_prev = 0

        self.wheel_diameter = 0.63
        self.wheel_circumference = self.wheel_diameter * 3.1416

        GPIO.add_event_detect(self.Sensor_hall_motor, GPIO.RISING, callback=self.read_engine_encoder, bouncetime=1)
        GPIO.add_event_detect(self.Sensor_hall_roda_A, GPIO.RISING, callback=self.read_wheel_encoder, bouncetime=1)

    def read_wheel_encoder(self, channel):
        a_state = GPIO.input(self.Sensor_hall_roda_A)
        b_state = GPIO.input(self.Sensor_hall_roda_B)
        increment = 0
        if a_state == GPIO.HIGH and b_state == GPIO.LOW:
            increment = 1
        elif a_state == GPIO.LOW and b_state == GPIO.HIGH:
            increment = -1
        self.wheel_pos_i += increment

    def compute_wheel_velocity(self):
        pos = self.wheel_pos_i

        current_time = time.time()
        elapsed_time = (current_time - self.previous_wheel_time)
        velocity_ps = (pos - self.pos_prev) / elapsed_time
        self.pos_prev = pos
        self.previous_wheel_time = current_time

        wheel_circumference = 0.63 * 3.1416

        velocity_kmh = velocity_ps * wheel_circumference * 3.6

        print(f"Velocidade Roda: {velocity_kmh} km/h") # Em pulsos/s

        return velocity_kmh

    def read_engine_encoder(self, channel):
        self.engine_pulse_count += 1

    def get_engine_rpm(self):
        current_time = time.time()
        elapsed_time = current_time - self.previous_engine_time
        self.previous_engine_time = current_time

        rpm = (self.engine_pulse_count / elapsed_time) * 60
        self.engine_pulse_count = 0
        return rpm

    def stop(self):
        GPIO.remove_event_detect(self.Sensor_hall_motor)
        GPIO.remove_event_detect(self.Sensor_hall_roda_A)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()