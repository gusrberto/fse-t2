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

        if accelerator and not brake:
            reference_speed = min(current_speed + 2, max_speed)
        elif brake and not accelerator:
            reference_speed = max(current_speed - 2, 0)
            self.apply_break_intensity(50)
        else:
            reference_speed = max(current_speed - 0.5, 0)

        return reference_speed
    
    def stop(self):
        self.brake_pwm.stop()
        GPIO.cleanup()