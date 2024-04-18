class PID:
    def __init__(self, CP, dt=0.05):
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
        self.integral_error = max(-5, min(self.integral_error, 5)) 

        pid = self.K_P*error + self.K_I*self.integral_error + self.K_D*derivative_error
        pid = max(-100, min(pid, 100))  
        self.pre_error = error
        
        return pid


# imported by JM
import matplotlib.pyplot as plt

class Apid:
    def __init__(self, CP, dt=0.05):
        self.Kp = 0.4
        self.Ki = 0.01
        self.Kd = 0.01

        self.dKp = 0
        self.dKi = 0
        self.dKd = 0

        self.ddKp = 0
        self.ddKi = 0
        self.ddKd = 0

        self.errs = [-1, -1, -1, -1, -1] # k-3, k-2, k-1, k, k+1 
        self.outs = [-1, -1, -1, -1, -1]
        self.ctrls = [-1, -1, -1, -1, -1]

        self.lr = 0.01

        self.cnt = 0

        self.ref = 10
        self.cur = 0
    
    def run(self, ref, cur):
        # update
        for i in range(3):
            self.errs[i] = self.errs[i+1]
            self.outs[i] = self.outs[i+1]
            self.ctrls[i] = self.ctrls[i+1]
        self.errs[3] = ref - cur
        self.outs[3] = cur
        self.ctrls[3] = self.ctrls[4]

        if self.cnt < 3:
            self.ctrls[4] = self.Kp*self.errs[3]
            self.cnt += 1
            Kp = self.Kp
            Ki = self.Ki
            Kd = self.Kd
        else:
            # ddK(k-1)
            self.ddKp = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / (self.ctrls[1] - self.ctrls[0])) * \
                        (self.errs[2] - self.errs[3])
            self.ddKi = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / (self.ctrls[1] - self.ctrls[0])) * \
                        (self.errs[2])
            self.ddKd = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / (self.ctrls[1] - self.ctrls[0])) * \
                        (self.errs[2] - 2*self.errs[1] + self.errs[0])
            # dK(k)
            self.dKp = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / (self.ctrls[2] - self.ctrls[1])) * \
                        (self.errs[3] - self.errs[2])
            self.dKi = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / (self.ctrls[2] - self.ctrls[1])) * \
                        (self.errs[3])
            self.dKd = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / (self.ctrls[2] - self.ctrls[1])) * \
                        (self.errs[3] - 2*self.errs[2] + self.errs[1])
            

            # K(k+1)
            Kp = self.Kp + self.dKp + self.ddKp
            Ki = self.Ki + self.dKi + self.ddKi
            Kd = self.Kd + self.dKd + self.ddKd
            # print("dP:{:4.2f} dI:{:4.2f} dD:{:4.2f}".format(self.dKp + self.ddKp, self.dKi + self.ddKi, self.Kd + self.dKd + self.ddKd))

            # control_amount
            delta_u_k = (Kp * (self.errs[3] - self.errs[2])) + \
                        (Ki * self.errs[3]) + \
                        (Kd * (self.errs[3] - 2*self.errs[2] + self.errs[1]))

            self.ctrls[4] = self.ctrls[3] + delta_u_k

        self.cur += self.ctrls[4]

        # print("P:{:4.2f} I:{:4.2f} D:{:4.2f}".format(Kp, Ki, Kd))

        return self.ctrls[4]

    def visualize(self):
        plt.ion()
        fig, ax = plt.subplots()
        t, ref_values, cur_values, ctrl_values = [], [], [], []
        cur_t = 0
        while 1:
            control = self.run(self.cur, self.ref)

            t.append(cur_t)
            cur_t += 0.05
            ref_values.append(self.ref)
            cur_values.append(self.cur)
            ctrl_values.append(control)

            if len(ref_values) > 30:
                t.pop(0)
                ref_values.pop(0)
                cur_values.pop(0)
                ctrl_values.pop(0)

            ax.clear()
            ax.plot(t, ref_values, label='Reference')
            ax.plot(t, cur_values, label='Current')
            # ax.plot(t, ctrl_values, label='Control', linestyle='--')
            ax.grid(True)
            ax.legend(loc='upper right')
            plt.xlabel('Time step')
            plt.ylabel('Value')
            plt.ylim(0, 12)
            plt.title('Real-time APID Control Visualization')
            plt.pause(0.01)  
        plt.ioff() 
        plt.show()
    
if __name__ == "__main__":

    apid = Apid()
    while 1:
        # apid.run(apid.cur, apid.ref)
        apid.visualize()