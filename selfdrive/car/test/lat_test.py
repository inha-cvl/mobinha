import can
import cantools
import threading
import time

class IONIQ:
    def __init__(self):
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/inha/Documents/catkin_ws/src/mobinha/selfdrive/car/dbc/ioniq/can.dbc')
        self.wheel_ang = 0
        self.temp_wheel = 0
        self.alv_cnt = 0
        self.reset = 0
        self.time = time.time()
        self.steering_overide = 0
        self.safety_status = 0
        self.enable = 0
        self.Gway_Steering_Tq = None
        self.Gway_Steering_Status = None
        self.gway_stat = None
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0, 0.05:0}

    def reset_trigger(self):
        self.reset = 1
        time.sleep(0.1)
        self.reset = 0
        # if self.reset:
        #     self.reset = 0
        # else:
        #     self.reset = 1

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False
        
    def daemon(self):
        while 1:
            self.wheel_ang_cmd()
            self.wheel_ang_rcv()
            # time.sleep(0.02)
    
    def mover(self):
        for i in range(0,1000, 5):
            self.wheel_ang = i/10
            time.sleep(0.02)

    def alive_counter(self, alv_cnt):
        alv_cnt += 1
        alv_cnt = 0 if alv_cnt > 255 else alv_cnt
        return alv_cnt
    
    def wheel_ang_cmd(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.enable, 'PA_StrAngCmd': self.wheel_ang,
                   'LON_Enable': 0, 'Target_Brake': 50, 'Target_Accel': 0, 
                   'Alive_cnt': self.alv_cnt, 'Reset_Flag': self.reset}
        msg = self.db.encode_message('Control', signals)
        # self.reset = 0
        self.sender(0x210, msg)

    def wheel_ang_rcv(self):
        data = self.bus.recv()
        if data.arbitration_id == 656:
            res = self.db.decode_message(656, data.data)
            self.temp_wheel = res['Gway_Steering_Angle']
        if data.arbitration_id == 784:
            res = self.db.decode_message(784, data.data)
            self.steering_overide = res['Steering_Overide']
            self.safety_status = res['Safety_Status']
        if data.arbitration_id == 3221225472:
            res = self.db.decode_message(3221225472, data.data)
            self.Gway_Steering_Tq = res['Gway_Steering_Tq']
            self.Gway_Steering_Status = res['Gway_Steering_Status']
            self.gway_stat = res['gway_stat']
        if self.timer(1):
            print(self.temp_wheel, " | ovr:", self.steering_overide, " | reset:", self.reset)
            # print("stat:", self.gway_stat," | Tq:", self.Gway_Steering_Tq, " | steer status:", self.Gway_Steering_Status)

    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def controller(self):
        while 1:
            if self.enable:
                print("ENABLE")
            else:
                print("DISABLE")
            cmd = input('99 == enable/disable 1001=reset\nenter tire angle : ')
            cmd = float(cmd)
            if -31 <= cmd <= 31:
                self.wheel_ang = float(cmd) * 13.73
            elif cmd == 1001:
                self.reset_trigger()
            elif cmd == 1000:
                exit(0)
            elif cmd == 1002:
                print('a')
                self.mover()
            elif cmd == 99:
                if self.enable == 0:
                    # self.wheel_ang = 0
                    self.enable = 1
                    # self.reset = 0
                elif self.enable == 1:
                    self.enable = 0

if __name__ == '__main__':
    IONIQ = IONIQ()
    t1 = threading.Thread(target=IONIQ.daemon)
    t2 = threading.Thread(target=IONIQ.controller)

    t1.start()
    t2.start()

    t1.join()
    t2.join()