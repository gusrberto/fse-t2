class PID:
    def __init__(self, kp=2.0, ki=0.3, kd=0.1, T=0.2):
        self.reference, self.measured_output, self.control_signal = 0.0, 0.0, 0.0
        self.kp = kp  # Proportional Gain
        self.ki = ki  # Integral Gain
        self.kd = kd  # Derivative Gain
        self.T = T  # Sampling Period
        self.total_error = 0.0
        self.previous_error = 0.0
        self.control_signal_max = 100.0
        self.control_signal_min = 0.0

    def refresh_reference(self, new_reference):
        self.reference = new_reference

    def controller(self, measured_output):
        error = self.reference - measured_output

        # Integral term (acumulado de erro com saturação)
        if abs(error) > 0.01:  # Evitar acumular erros muito pequenos
            self.total_error += error * self.T

        # Evitar saturação da integral
        self.total_error = max(
            self.control_signal_min / (self.ki + 1e-6), 
            min(self.control_signal_max / (self.ki + 1e-6), self.total_error)
        )

        # Derivative term
        delta_error = (error - self.previous_error) / self.T

        # PID control signal
        self.control_signal = (
            self.kp * error
            + self.ki * self.total_error
            + self.kd * delta_error
        )

        # Saturação do controle
        self.control_signal = max(self.control_signal_min, min(self.control_signal_max, self.control_signal))

        # Atualizar erro anterior
        self.previous_error = error

        return self.control_signal
