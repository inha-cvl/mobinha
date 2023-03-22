class PID:
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
        diff_error = (error - self.pre_error)/self.dt
        self.pre_error = error # *self.dt
        self.error_sum += error
        if self.error_sum < -5:
            self.error_sum = -5
        elif self.error_sum > 5:
            self.error_sum = 5
        
        pid = self.K_P*error + self.K_D*diff_error + self.K_I*self.error_sum*self.dt
        
        return pid
    
'''
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
        self.integral_error += error*self.dt

        if self.integral_error < -5:
            self.integral_error = -5
        elif self.integral_error > 5:
            self.integral_error = 5
        
        pid = self.K_P*error + self.K_I*self.integral_error + self.K_D*derivative_error
        self.pre_error = error
        return pid
'''