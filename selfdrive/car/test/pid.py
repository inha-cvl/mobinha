                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              class PID:
    def __init__(self):
        self.P = 0.4
        self.I = 0.0002
        self.D = 0.002

        #self.P = 1.92
        #self.I = 1.25
        #self.D = 0.31
        
        self.pre_error = 0.0
        self.error_sum = 0.0
        self.dt = 1.0 / 10.0

    def run(self, target, current):
        if target == 0.0 and current < 1.0 / 3.6:
            self.pre_error = 0.0
            self.error_sum = 0.0
            return -0.7
        else:
            error = target - current
            diff_error = error - self.pre_error
            self.pre_error = error
            self.error_sum += error
            return self.P*error + self.D*diff_error/self.dt + self.I*self.error_sum*self.dt
