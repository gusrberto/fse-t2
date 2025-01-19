import RPi.GPIO as GPIO

class VehicleControl:
    def __init__(self, max_speed=200):
        self.current_target_speed = 0
        self.max_speed = max_speed
        self.acceleration_step = 2
        self.brake_step = 5
        self.inertia_step = 1

        # Configuração dos GPIOs
        self.Motor_DIR1 = 17
        self.Motor_DIR2 = 18
        self.Motor_POT = 23
        self.Freio_INT = 24
        self.Pedal_AC = 27
        self.Pedal_FR = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.Motor_DIR1, self.Motor_DIR2, self.Motor_POT, self.Freio_INT], GPIO.OUT)
        GPIO.setup([self.Pedal_AC, self.Pedal_FR], GPIO.IN)

        self.engine_pwm = GPIO.PWM(self.Motor_POT, 1000)  # Frequência de 1kHz
        self.brake_pwm = GPIO.PWM(self.Freio_INT, 1000)  # Frequência de 1kHz
        self.engine_pwm.start(0)
        self.brake_pwm.start(0)

    def calculate_target_speed(self, pedal_ac, pedal_fr):
        if pedal_ac and not pedal_fr:
            self.current_target_speed = min(self.current_target_speed + self.acceleration_step, self.max_speed)
        elif pedal_fr and not pedal_ac:
            self.current_target_speed = max(self.current_target_speed - self.brake_step, 0)
        elif not pedal_ac and not pedal_fr:
            self.current_target_speed = max(self.current_target_speed - self.inertia_step, 0)
        elif pedal_ac and pedal_fr:
            self.current_target_speed = max(self.current_target_speed - self.brake_step, 0)

        return self.current_target_speed

    def read_inputs(self):
        # Leitura dos pedais
        pedal_ac = GPIO.input(self.Pedal_AC)
        pedal_fr = GPIO.input(self.Pedal_FR)
        print(f"Acelerador: {pedal_ac} | Freio: {pedal_fr}")
        return pedal_ac, pedal_fr

    def set_engine_mode(self, dir1, dir2, engine_pwm_value, brake_pwm_value):
        # Configuração das saídas
        GPIO.output(self.Motor_DIR1, dir1)
        GPIO.output(self.Motor_DIR2, dir2)
        self.engine_pwm.ChangeDutyCycle(engine_pwm_value)
        self.brake_pwm.ChangeDutyCycle(brake_pwm_value)

    def engine_controller(self, pid_control_signal=50):
        pedal_ac, pedal_fr = self.read_inputs()

        brake_pwm_value = 50

        if pedal_ac and not pedal_fr:
            # Andar pra frente
            self.set_engine_mode(GPIO.HIGH, GPIO.LOW, pid_control_signal, 0)
        elif pedal_fr and not pedal_ac:
            # Freio
            self.set_engine_mode(GPIO.HIGH, GPIO.HIGH, 0, brake_pwm_value)
        elif pedal_ac and pedal_fr:
            # Prioridade do freio
            self.set_engine_mode(GPIO.HIGH, GPIO.HIGH, 0, brake_pwm_value)
        else:
            # Neutro
            self.set_engine_mode(GPIO.LOW, GPIO.LOW, 0, 0)

    def cleanup(self):
        self.engine_pwm.stop()
        self.brake_pwm.stop()
        GPIO.cleanup()        