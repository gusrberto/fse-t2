import RPi.GPIO as GPIO

class Engine():
    def __init__(self) -> None:
        self.Motor_DIR1 = 17
        self.Motor_DIR2 = 18
        self.Motor_POT_PWM = 23

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.Motor_DIR1, GPIO.OUT)
        GPIO.setup(self.Motor_DIR2, GPIO.OUT)
        GPIO.setup(self.Motor_POT_PWM, GPIO.OUT)

        self.pwm = GPIO.PWM(self.Motor_POT_PWM, 1000)
        self.pwm.start(0)

    def accelerate_forward(self):
        GPIO.output(self.Motor_DIR1, GPIO.HIGH)
        GPIO.output(self.Motor_DIR2, GPIO.LOW)

    def accelerate_reverse(self):
        GPIO.output(self.Motor_DIR1, GPIO.LOW)
        GPIO.output(self.Motor_DIR2, GPIO.HIGH)

    def brake(self):
        GPIO.output(self.Motor_DIR1, GPIO.HIGH)
        GPIO.output(self.Motor_DIR2, GPIO.HIGH)

    def setDutyCycle(self, value):
        self.pwm.ChangeDutyCycle(value)
        
    def moveEngine(self, value):
        value = max(-100, min(100, value))
        self.setDutyCycle(abs(value))
        if value < 0:
            self.accelerate_reverse()
        elif value > 0:
            self.accelerate_forward()
        else:
            self.brake()     