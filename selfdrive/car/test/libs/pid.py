import numpy as np

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
        # 일반형
        self.window_size = 2
        self.Kp = 46
        self.Ki = 1.2 / self.window_size
        self.Kd = 35
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

        self.epsilon = 1e-8

        self.cnt = 0

        self.ref = None
        self.ref_prev = None

        self.accel_default = 10
        self.brake_default = 0.5
        self.accel_min = self.accel_default
        self.brake_min = self.brake_default
        self.accel_lim = self.accel_default
        self.brake_lim = self.brake_default

    
    def run(self, cur, ref):
        # update
        if self.ref is None:
            self.ref = ref
        if self.ref != ref:
            self.ref_prev = self.ref
            self.ref = ref
        self.cur_v = cur

        self.accel_min = self.accel_default + 1.2*cur/10
        self.brake_min = self.brake_default

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
        self.derivative = self.errs[3]-self.errs[2]

        output = (Kp * self.error) + (Ki * self.integral) + (Kd *self.derivative)

        accel_val, brake_val = self.post_process(output)
        
        return accel_val, brake_val

        
    def post_process(self, output):
        # print("                      ACCEL_MIN :", self.accel_min)
        accel_lim_max = 33
        brake_lim_max = 45

        self.v_err = self.ref - self.cur_v

        if self.ref_prev is not None and self.ref is not None:
            # accel
            if self.ref_prev < self.ref:
                self.brake_lim -= 0.2

                if self.cur_v < self.ref_prev + 0.4*(self.ref - self.ref_prev):
                    self.accel_lim += 0.25
                    print("[ACCEL] mode 1 : accel limit increase")
                    
                elif self.ref_prev + 0.7*(self.ref - self.ref_prev) < self.cur_v < self.ref:
                    self.accel_lim -= 0.3
                    print("[ACCEL] mode 3 : accel limit decrease")
            
                else:
                    print("[ACCEL] mode 2 : maintain")
                    
            

            # brake
            if self.ref_prev > self.ref and self.cur_v > self.ref:
                self.accel_lim -= 0.2

                if self.cur_v > self.ref_prev + 0.4*(self.ref - self.ref_prev):
                    self.brake_lim += 0.3
                    print("[BRAKE] mode 1 : brake limit increase", self.brake_lim)
                    
                elif self.ref_prev + 0.7*(self.ref - self.ref_prev) > self.cur_v > self.ref:
                    self.brake_lim -= 0.3
                    print("[BRAKE] mode 3 : brake limit decrease")
                else:
                    print("[BRAKE] mode 2 : maintain")
            

            self.accel_lim = np.clip(self.accel_lim, self.accel_min, accel_lim_max)
            self.brake_lim = np.clip(self.brake_lim, self.brake_min, brake_lim_max)


            self.v_err_prev = self.v_err

            print("                                 ACC_LIM, BRK_LIM : ", self.accel_lim, self.brake_lim)

        else:
            print("initialize")

        if output>0:
            accel_val = min(output, self.accel_lim)
            brake_val = 0
        else:
            accel_val = 0
            brake_val = min(-output, self.brake_lim)
        
        if self.ref == 0 and self.cur_v < 2.5:
            brake_val = 40

        return accel_val, brake_val