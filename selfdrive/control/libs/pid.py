class PID:
    def __init__(self, CP, dt=0.1):
        self.K_P = CP.kpV
        self.K_I = CP.kiV
        self.K_D = CP.kf
        self.pre_error = 0.0
        self.integral_error = 0.0
        self.dt = dt

    def run(self, target, current):
        error = target - current
        derivative_error = (error - self.pre_error)/self.dt
        self.integral_error += error
        self.integral_error = max(-5, min(self.integral_error, 5))

        pid = self.K_P*error + self.K_I*self.integral_error*self.dt + self.K_D*derivative_error
        self.pre_error = error
        return pid