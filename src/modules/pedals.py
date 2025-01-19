import RPi.GPIO as GPIO

class Pedals:
    def __init__(self):
        self.Pedal_AC = 27
        self.Pedal_FR = 22
        self.Freio_INT = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.Pedal_AC, GPIO.IN)
        GPIO.setup(self.Pedal_FR, GPIO.IN)
        GPIO.setup(self.Freio_INT, GPIO.OUT)

        self.brake_pwm = GPIO.PWM(self.Freio_INT, 1000)
        self.brake_pwm.start(0)

    def read_accelerator(self):
        return GPIO.input(self.Pedal_AC)
    
    def read_brake(self):
        return GPIO.input(self.Pedal_FR)
    
    def apply_break_intensity(self, intensity):
        intensity = max(0, min(100, intensity))
        self.brake_pwm.ChangeDutyCycle(intensity)

    def calculate_reference_speed(self, current_speed, max_speed=100):
        accelerator = self.read_accelerator()
        brake = self.read_brake()

        print(f"Acelerador: {accelerator} || Freio: {brake}")

        if accelerator and not brake:
            reference_speed = min(current_speed + 10, max_speed)
        elif brake and not accelerator:
            reference_speed = max(current_speed - 10, 0)
            self.apply_break_intensity(50)
        else:
            reference_speed = max(current_speed - 1, 0)

        return reference_speed
    
    def check_pedals(self):
        try:
            # Verifica se os pinos foram configurados corretamente
            GPIO.setup(self.Pedal_AC, GPIO.IN)
            GPIO.setup(self.Pedal_FR, GPIO.IN)

            # Faz leituras para garantir que estão acessíveis
            accelerator_state = self.read_accelerator()
            brake_state = self.read_brake()

            print(f"Estado inicial do acelerador: {accelerator_state}, Estado inicial do freio: {brake_state}")

            return True
        except Exception as e:
            print(f"Erro ao verificar pedais: {e}")
            return False

    def stop(self):
        self.brake_pwm.stop()
        GPIO.cleanup()