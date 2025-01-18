class PID:
    def __init__(self, kp=0.5, ki=0.05, kd=40.0, T=0.2):
        self.reference, self.measured_output, self.control_signal = 0.0, 0.0, 0.0
        self.kp = kp # Proportional Gain
        self.ki = ki # Integral Gain
        self.kd = kd # Derivative Gain
        self.T = T # Sampling Period
        self.last_time = 0
        self.total_error = 0.0
        self.previous_error = 0.0
        self.control_signal_max = 100.0
        self.control_signal_min = -100.0

    def refresh_reference(self, new_reference):
        self.reference = new_reference

    def controller(self, measured_output):
        error = self.reference - measured_output

        self.total_error += error # integral term

        if self.total_error >= self.control_signal_max:
            self.total_error = self.control_signal_max
        elif self.total_error <= self.control_signal_min:
            self.total_error = self.control_signal_min

        delta_error = error - self.previous_error # derivative term

        self.control_signal = (self.kp * error + (self.ki * self.T) * self.total_error + (self.kd / self.T) * delta_error)

        if self.control_signal >= self.control_signal_max:
            self.control_signal = self.control_signal_max
        elif self.control_signal <= self.control_signal_min:
            self.control_signal = self.control_signal_min

        self.previous_error = error

        return self.control_signal 