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
        self.window_size = 4
        self.Kp = 44
        self.Ki = 1.5 / self.window_size
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

        self.accel_lim = 10
        self.brake_lim = 15


    # def limit_steer_change(self, current_accel_lim):
    #     saturation_th = 5
    #     saturated_accel_lim = current_accel_lim
    #     diff = max(min(current_accel_lim-self.prev_accel_lim, saturation_th), -saturation_th)
    #     saturated_accel_lim = self.prev_accel_lim + diff
    #     return saturated_accel_lim
    
    def run(self, cur, ref):
        # update
        if self.ref is None:
            self.ref = ref
        if self.ref != ref:
            self.ref_prev = self.ref
            self.ref = ref
        self.cur_v = cur

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
        accel_lim_max = 30
        brake_lim_max = 30

        # 속도에러 미분항
        self.v_err = self.ref - self.cur_v

        if self.ref_prev is not None and self.ref is not None:
            if self.ref_prev < self.ref:
                if self.cur_v < self.ref_prev + 0.3*(self.ref - self.ref_prev):
                    self.accel_lim += 0.1
                    print("mode 1")
                    
                elif self.ref_prev + 0.7*(self.ref - self.ref_prev) < self.cur_v < self.ref - 2/3.6:
                    print("mode 2")
                    if self.v_err_prev - self.v_err < 0.02:
                        print("mode 2:pass")
                        pass
                    else:
                        self.accel_lim -= 0.1
            
            self.accel_lim = np.clip(self.accel_lim, 0, accel_lim_max)
            self.v_err_prev = self.v_err

            print(self.accel_lim)

            if self.ref_prev > self.ref:
                if self.cur_v > self.ref_prev + 0.3*(self.ref - self.ref_prev):
                    self.brake_lim += 0.01
                if self.ref_prev + 0.7*(self.ref - self.ref_prev) > self.cur_v > self.ref:
                    self.brake_lim -= 0.01

            self.brake_lim = np.clip(self.brake_lim, 0, brake_lim_max)
        else:
            print("initialize")
            # self.accel_lim = accel_lim_max
            # self.brake_lim = brake_lim_max
            

        if output>0:
            accel_val = min(output, self.accel_lim)
            brake_val = 0
        else:
            accel_val = 0
            brake_val = min(-output, self.brake_lim)

        return accel_val, brake_val


        # if cur*3.6 < 54:
        #     accel_lim = 1.3*cur+10