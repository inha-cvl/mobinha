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
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0}

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
            time.sleep(0.02)
    
    def mover(self):
        for i in range(0,1000, 5):
            self.wheel_ang = i/10
            time.sleep(0.02)
            
    def wheel_ang_cmd(self):
        signals = {'PA_Enable': self.enable, 'PA_StrAngCmd': self.wheel_ang,
                   'LON_Enable': 0, 'Target_Brake': 1, 'Target_Accel': 0, 'Alive_cnt': 0 if self.alv_cnt > 255 else self.alv_cnt , 'Reset_Flag': self.reset}
        msg = self.db.encode_message('Control', signals)
        self.sender(0x210, msg)
        self.reset = 0
        self.alv_cnt += 1

    def wheel_ang_rcv(self):
        if self.timer(1):
            print(self.temp_wheel, " | ", self.steering_overide, " | ", self.safety_status)
        data = self.bus.recv()
        if data.arbitration_id == 656:
            res = self.db.decode_message(656, data.data)
            self.temp_wheel = res['Gway_Steering_Angle']
        if data.arbitration_id == 784:
            res = self.db.decode_message(784, data.data)
            self.steering_overide = res['Steering_Overide']
            self.safety_status = res['Safety_Status']
               

    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def controller(self):
        while 1:
            print(self.enable)
            cmd = input('1 == enable/disable 1001=reset\nenter angle : ')
            cmd = int(cmd)
            if 1 < cmd < 30:
                self.wheel_ang = float(cmd) * 13.73
            elif cmd == 1001:
                self.reset = 1 
            elif cmd == 1000:
                exit(0)
            elif cmd == 1002:
                print('a')
                self.mover()
            elif cmd == 1:
                if self.enable == 0:
                    self.enable = 1
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