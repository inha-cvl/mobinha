class PID:
    def __init__(self, dt=0.05):
        self.K_P = 2.98
        self.K_I = 2.38
        self.K_D = 0.00
        self.pre_error = 0.0
        self.integral_error = 0.0
        self.dt = dt

    def run(self, target, current):
        error = target - current 

        derivative_error = (error - self.pre_error)/self.dt 
        self.integral_error += error*self.dt
        self.integral_error = max(-5, min(self.integral_error, 5)) 

        pid = self.K_P*error + self.K_I*self.integral_error + self.K_D*derivative_error
        pid = max(-100, min(pid, 100))  
        self.pre_error = error
        
        if pid>0:
            accel_val = pid
            brake_val = 0
        else:
            accel_val = 0
            brake_val = pid

        return accel_val, brake_val


# imported by JM
import matplotlib.pyplot as plt

class APID:
    def __init__(self):
        # 증분형
        # self.Kp = 8
        # self.Ki = 0.004
        # self.Kd = 10
        # self.lr = 0.001 # 0.0001

        # 일반형
        self.window_size = 4
        self.Kp = 40
        self.Ki = 12 / self.window_size
        self.Kd = 20
        self.lr = 0.001 # 0.0001
        self.error_history = []

        self.dKp = 0
        self.dKi = 0
        self.dKd = 0

        self.ddKp = 0
        self.ddKi = 0
        self.ddKd = 0

        self.errs = [-1, -1, -1, -1, -1] # k-3, k-2, k-1, k, k+1 
        self.outs = [-1, -1, -1, -1, -1]
        self.ctrls = [-1, -1, -1, -1, -1]


        self.cnt = 0

        self.ref = 10
        self.cur = 0

        self.epsilon = 1e-8
    
    def run(self, cur, ref):
        # update
        for i in range(3):
            self.errs[i] = self.errs[i+1]
            self.outs[i] = self.outs[i+1]
            self.ctrls[i] = self.ctrls[i+1]
        self.errs[3] = ref - cur
        self.outs[3] = cur
        self.ctrls[3] = self.ctrls[4]
        self.error_history.append(ref - cur)
        if len(self.error_history)>self.window_size:
            self.error_history.pop(0)

        # ddK(k-1)
        tmp1 = (self.ctrls[1] - self.ctrls[0] + self.epsilon)
        
        self.ddKp = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] - self.errs[3] + self.epsilon)
        self.ddKi = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] + self.epsilon)
        self.ddKd = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] - 2*self.errs[1] + self.errs[0] + self.epsilon)
        # dK(k)
        tmp2 = (self.errs[3] - self.errs[2])
        if tmp2 == 0:
            tmp2 = 1000 

        self.dKp = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] - self.errs[2] + self.epsilon)
        self.dKi = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] + self.epsilon)
        self.dKd = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] - 2*self.errs[2] + self.errs[1] + self.epsilon)
        
        # clip 
        plim = self.Kp/10
        ilim = self.Ki/10
        dlim = self.Kd/10
        self.ddKp = max(-plim, min(self.ddKp, plim))
        self.ddKi = max(-ilim, min(self.ddKi, plim))
        self.ddKd = max(-dlim, min(self.ddKd, plim))
        self.dKp = max(-plim, min(self.dKp, plim))
        self.dKi = max(-ilim, min(self.dKi, plim))
        self.dKd = max(-dlim, min(self.dKd, plim))
        
        # K(k+1)
        Kp = self.Kp + self.dKp + self.ddKp
        Ki = self.Ki + self.dKi + self.ddKi
        Kd = self.Kd + self.dKd + self.ddKd

        # control_amount : 일반형
        self.error = self.errs[3]
        self.integral = sum(self.error_history)
        # if self.errs[3] < 2:
        #     self.integral = 0
        self.derivative = self.errs[3]-self.errs[2]

        output = (Kp * self.error) + (Ki * self.integral) + (Kd *self.derivative)
        self.cur += output
        # control_amount : 증분형
        # u_k_p = Kp * (self.errs[3] - self.errs[2])
        # lim = 0.1
        # u_k_i = max(-lim, min((Ki * self.errs[3]), lim))
        # u_k_d = Kd * (self.errs[3] - 2*self.errs[2] + self.errs[1])
        # delta_u_k = u_k_p + u_k_i + u_k_d 
        # delta_u_k = (Kp * (self.errs[3] - self.errs[2])) + \
        #             (Ki * self.errs[3]) + \
        #             (Kd * (self.errs[3] - 2*self.errs[2] + self.errs[1]))
        # self.ctrls[4] = self.ctrls[3] + delta_u_k
        # self.cur += self.ctrls[4]

        # print("P:{:4.2f} I:{:4.2f} D:{:4.2f}".format(Kp, Ki, Kd))
        
        # output = max(-100, min(self.ctrls[4], 100))
        # if output>0:
        #     accel_val = output
        #     brake_val = 0
        # else:
        #     accel_val = 0
        #     brake_val = -output
        # output = max(-100, min(self.ctrls[4], 100))

        accel_lim = 80
        brake_lim = 50
        if output>0:
            accel_val = min(output, accel_lim)
            brake_val = 0
        else:
            accel_val = 0
            brake_val = min(-output, brake_lim)

        # print(accel_val)
        return accel_val, brake_val
        # return self.ctrls[4]