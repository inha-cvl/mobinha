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
        self.Kp = 8
        self.Ki = 0.004
        self.Kd = 10

        self.dKp = 0
        self.dKi = 0
        self.dKd = 0

        self.ddKp = 0
        self.ddKi = 0
        self.ddKd = 0

        self.errs = [-1, -1, -1, -1, -1] # k-3, k-2, k-1, k, k+1 
        self.outs = [-1, -1, -1, -1, -1]
        self.ctrls = [-1, -1, -1, -1, -1]

        self.lr = 0.001 # 0.0001

        self.cnt = 0

        self.ref = 10
        self.cur = 0

        self.epsilon = 1e-4
    
    def run(self, ref, cur):
        # update
        for i in range(3):
            self.errs[i] = self.errs[i+1]
            self.outs[i] = self.outs[i+1]
            self.ctrls[i] = self.ctrls[i+1]
        self.errs[3] = ref - cur
        self.outs[3] = cur
        self.ctrls[3] = self.ctrls[4]

        # if self.cnt < 3:
        #     self.ctrls[4] = self.Kp*self.errs[3]
        #     self.cnt += 1
        #     Kp = self.Kp
        #     Ki = self.Ki
        #     Kd = self.Kd
        # else:
    #     # ddK(k-1)
        if (self.ctrls[1] - self.ctrls[0]) == 0:
            tmp1 = 1000
        else:
            tmp1 = (self.ctrls[1] - self.ctrls[0])

        self.ddKp = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] - self.errs[3])
        self.ddKi = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2])
        self.ddKd = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] - 2*self.errs[1] + self.errs[0])
        # dK(k)
        if (self.ctrls[2] - self.ctrls[1]) == 0:
            tmp2 = 1000
        else:
            tmp2 = (self.ctrls[2] - self.ctrls[1])

        self.dKp = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] - self.errs[2])
        self.dKi = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3])
        self.dKd = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] - 2*self.errs[2] + self.errs[1])
        

        # K(k+1)
        tp = self.dKp + self.ddKp
        ti = self.dKi + self.ddKi
        td = self.dKd + self.ddKd
        Kp = self.Kp + max(-self.Kp/10, min(tp, self.Kp/10))
        Ki = self.Ki + max(-self.Ki/10, min(ti, self.Ki/10))
        Kd = self.Kd + max(-self.Kd/10, min(td, self.Kd/10))
        # Kp = self.Kp + self.dKp + self.ddKp
        # Ki = self.Ki + self.dKi + self.ddKi
        # Kd = self.Kd + self.dKd + self.ddKd
        # print("dP:{:4.2f} dI:{:4.2f} dD:{:4.2f}".format(self.dKp + self.ddKp, self.dKi + self.ddKi, self.Kd + self.dKd + self.ddKd))

        # control_amount
        u_k_p = Kp * (self.errs[3] - self.errs[2])
        lim = 0.1
        u_k_i = max(-lim, min((Ki * self.errs[3]), lim))
        u_k_d = Kd * (self.errs[3] - 2*self.errs[2] + self.errs[1])
        
        # bias
        if self.errs[3] < -1:
            affine = -0.01
        elif self.errs[3] > 1:
            affine = 0.01
        else:
            affine = 0
        delta_u_k = u_k_p + u_k_i + u_k_d + affine
        # print(u_k_p, u_k_i, u_k_d)
        # print(u_k_i)
        # delta_u_k = (Kp * (self.errs[3] - self.errs[2])) + \
        #             (Ki * self.errs[3]) + \
        #             (Kd * (self.errs[3] - 2*self.errs[2] + self.errs[1]))

        self.ctrls[4] = self.ctrls[3] + delta_u_k

        self.cur += self.ctrls[4]

        # print("P:{:4.2f} I:{:4.2f} D:{:4.2f}".format(Kp, Ki, Kd))
        
        apid = max(-100, min(self.ctrls[4], 100))
        if apid>0:
            accel_val = apid
            brake_val = 0
        else:
            accel_val = 0
            brake_val = -apid

        return accel_val, brake_val
        # return self.ctrls[4]