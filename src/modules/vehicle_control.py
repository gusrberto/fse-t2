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

        # Configuração das luzes/faróis
        self.Farol_Baixo = 19
        self.Farol_Alto = 26
        self.Luz_Freio = 25
        self.Luz_Seta_Esq = 8
        self.Luz_Seta_Dir = 7
        self.Luz_Temp_Motor = 12

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.Pedal_AC, GPIO.IN)
        GPIO.setup(self.Pedal_FR, GPIO.IN)

        GPIO.setup([self.Motor_DIR1, self.Motor_DIR2, self.Motor_POT, self.Freio_INT], GPIO.OUT)
        GPIO.setup([self.Farol_Baixo, self.Farol_Alto, self.Luz_Freio, self.Luz_Seta_Dir, self.Luz_Seta_Esq, self.Luz_Temp_Motor], GPIO.OUT)

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
    
    def read_accelerator_pedal(self):
        return GPIO.input(self.Pedal_AC)
    
    def read_brake_pedal(self):
        return GPIO.input(self.Pedal_FR)

    def set_engine_mode(self, dir1, dir2, engine_pwm_value, brake_pwm_value):
        # Configuração das saídas
        GPIO.output(self.Motor_DIR1, dir1)
        GPIO.output(self.Motor_DIR2, dir2)
        self.engine_pwm.ChangeDutyCycle(engine_pwm_value)
        self.brake_pwm.ChangeDutyCycle(brake_pwm_value)

    def control_lights(self, farol_baixo=False, farol_alto=False, freio=False, seta_esq=False, seta_dir=False, motor_temp_alert=False):
        GPIO.output(self.Farol_Baixo, GPIO.HIGH if farol_baixo else GPIO.LOW)
        GPIO.output(self.Farol_Alto, GPIO.HIGH if farol_alto else GPIO.LOW)
        GPIO.output(self.Luz_Freio, GPIO.HIGH if freio else GPIO.LOW)
        GPIO.output(self.Luz_Seta_Esq, GPIO.HIGH if seta_esq else GPIO.LOW)
        GPIO.output(self.Luz_Seta_Dir, GPIO.HIGH if seta_dir else GPIO.LOW)
        GPIO.output(self.Luz_Temp_Motor, GPIO.HIGH if motor_temp_alert else GPIO.LOW)

    def update_lights_state(self):
        farol_baixo_estado = GPIO.input(self.Farol_Baixo)
        farol_alto_estado = GPIO.input(self.Farol_Alto)
        seta_esq_estado = GPIO.input(self.Luz_Seta_Esq)
        seta_dir_estado = GPIO.input(self.Luz_Seta_Dir)

        self.control_lights(
            farol_baixo=farol_baixo_estado,
            farol_alto=farol_alto_estado,
            seta_esq=seta_esq_estado,
            seta_dir=seta_dir_estado                
        )

        print(f"FB: {farol_baixo_estado}, FA: {farol_alto_estado}, SE: {seta_esq_estado}, SD: {seta_dir_estado}")

    def engine_controller(self, pid_control_signal=50):
        pid_control_signal = max(0, min(100, pid_control_signal))

        pedal_ac = self.read_accelerator_pedal()
        pedal_fr = self.read_brake_pedal()

        brake_pwm_value = 50

        if pedal_fr:
            self.control_lights(freio=True, farol_baixo=True, farol_alto=True, seta_dir=True, seta_esq=True)
        else:
            self.control_lights(freio=False, farol_baixo=False)

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