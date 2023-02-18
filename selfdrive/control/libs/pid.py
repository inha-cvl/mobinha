class PID:
    KPH_TO_MPS = 1 / 3.6
    MPS_TO_KPH = 3.6

    def __init__(self, CP, dt=0.1):
        self.K_P = CP.kpV
        self.K_I = CP.kiV
        self.K_D = CP.kf
        self.pre_error = 0.0
        self.error_sum = 0.0
        self.dt = dt

    def run(self, target, current):
        # if target < 0.1 and current < 2.5 / 3.6:
        #     self.pre_error = 0.0
        #     self.error_sum = 0.0
        #     return -0.9
        # else:
        error = target - current
        diff_error = error - self.pre_error
        self.pre_error = error
        self.error_sum += error
        if self.error_sum < -5:
            self.error_sum = -5
        elif self.error_sum > 5:
            self.error_sum = 5
        
        pid = self.K_P*error + self.K_D*diff_error/self.dt + \
        self.K_I*self.error_sum*self.dt
        
        return pid
