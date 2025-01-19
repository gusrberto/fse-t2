import RPi.GPIO as GPIO
import time

# Configuração dos GPIOs
Pedal_AC = 27
Pedal_FR = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(Pedal_AC, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(Pedal_FR, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        pedal_ac = GPIO.input(Pedal_AC)
        pedal_fr = GPIO.input(Pedal_FR)
        print(f"Pedal_AC: {pedal_ac}, Pedal_FR: {pedal_fr}")
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
