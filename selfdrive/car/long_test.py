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
        self.brake = 50
        self.alv_cnt = 0
        self.reset = 0
        self.time = time.time()
        self.acc_override = 0
        self.brk_override = 0
        self.safety_status = 0

        self.Gway_Accel_Pedal_Position = 0
        self.Gway_GearSelDisp = 0
        self.Gway_Brake_Active = 0
        self.Gway_Brake_Cylinder_Pressure = 0
        
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0}

    def reset_trigger(self):
        self.reset = 1
        time.sleep(1.5)
        self.reset = 0

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False
        
    def daemon(self):
        while 1:
            self.longitudinal_cmd()
            self.longitudinal_rcv()
            # time.sleep(0.02)

    def alive_counter(self, alv_cnt):
        alv_cnt += 1
        alv_cnt = 0 if alv_cnt > 255 else alv_cnt
        return alv_cnt
    
    def longitudinal_cmd(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': 0, 'PA_StrAngCmd': 0,
                   'LON_Enable': self.enable, 'Target_Brake': self.brake, 'Target_Accel': self.accel, 
                   'Alive_cnt': self.alv_cnt , 'Reset_Flag': self.reset}
        msg = self.db.encode_message('Control', signals)
        self.sender(0x210, msg)
        self.reset = 0

    def longitudinal_rcv(self):
        data = self.bus.recv()
        if data.arbitration_id == 368:
            res = self.db.decode_message(368, data.data)
            self.Gway_Accel_Pedal_Position = res['Gway_Accel_Pedal_Position']
            self.Gway_GearSelDisp = res['Gway_GearSelDisp']
        if data.arbitration_id == 608:
            res = self.db.decode_message(608, data.data)
            self.Gway_Brake_Active = res['Gway_Brake_Active']
        if data.arbitration_id == 304:
            res = self.db.decode_message(304, data.data)
            self.Gway_Brake_Cylinder_Pressure = res['Gway_Brake_Cylinder_Pressure']
        if data.arbitration_id == 784:
            res = self.db.decode_message(784, data.data)
            self.acc_override = res['Accel_Override']
            self.brk_override = res['Brake_Override']
            self.safety_status = res['Safety_Status']
        if self.timer(1):
            print("gear:", self.Gway_GearSelDisp, " | brake_active:", self.Gway_Brake_Active)
            print("acc:", self.Gway_Accel_Pedal_Position, " | brk:", self.Gway_Brake_Cylinder_Pressure)
            print("ovr(acl,brk):", self.acc_override, " | ", self.brk_override, " | reset:", self.reset)

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
            cmd = input('99 == enable/disable 1001=reset\naccel:1~5, brake:-1~-20\n')
            cmd = int(cmd)
            if 1 <= cmd <= 5:
                self.accel = float(cmd)*5
                self.brake = 0
            elif -20 <= cmd <= -1:
                self.brake = -float(cmd)*5
                self.accel = 0
            elif cmd == 99:
                if self.enable == 0:
                    self.accel = 0
                    self.brake = 50
                    self.enable = 1
                elif self.enable == 1:
                    self.enable = 0
                    self.reset_trigger()     
            elif cmd == 1001:
                self.reset_trigger()
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