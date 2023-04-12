import can
import cantools
import threading
import time

class IONIQ:
    def __init__(self):
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/inha/Documents/catkin_ws/src/mobinha/selfdrive/car/dbc/ioniq/can.dbc')
        self.accel = 0
        self.enable = 0
        self.brake = 1
        self.alv_cnt = 0
        self.reset = 0
        self.time = time.time()

    def daemon(self):
        while 1:
            self.longitudinal_cmd()
            time.sleep(0.02)
    
    def mover(self):
        for i in range(0,1000, 5):
            self.wheel_ang = i/10
            time.sleep(0.02)
            
    def longitudinal_cmd(self):
        signals = {'PA_Enable': 0, 'PA_StrAngCmd': 0,
                   'LON_Enable': self.enable, 'Target_Brake': self.brake, 'Target_Accel': self.accel, 'Alive_cnt': 0 if self.alv_cnt > 255 else self.alv_cnt , 'Reset_Flag': self.reset}
        msg = self.db.encode_message('Control', signals)
        self.sender(0x210, msg)
        self.reset = 0
        self.alv_cnt += 1

    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def controller(self):
        while 1:
            print(self.enable)
            cmd = input('1 == enable/disable 1001=reset\n1<accel<10, -1>brake>-10\n')
            cmd = int(cmd)
            if 1 < cmd < 10:
                self.accel = float(cmd)*3
                self.brake = 0
            elif -40 < cmd < -1:
                self.brake = -float(cmd)*3
                self.accel = 0
            elif cmd == 1:
                if self.enable == 0:
                    self.enable = 1
                elif self.enable == 1:
                    self.enable = 0
                    
            elif cmd == 1001:
                self.reset = 1
            elif cmd == 1000:
                exit(0)

if __name__ == '__main__':
    IONIQ = IONIQ()
    t1 = threading.Thread(target=IONIQ.daemon)
    t2 = threading.Thread(target=IONIQ.controller)

    t1.start()
    t2.start()

    t1.join()
    t2.join()