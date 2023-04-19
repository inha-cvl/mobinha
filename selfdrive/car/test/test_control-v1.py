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
        self.brake = 0
        self.temp_wheel = 0
        self.alv_cnt = 0
        self.reset = 0
        self.time = time.time()
        self.acc_override = None
        self.brk_override = None
        self.steering_overide = None
        self.safety_status = None

        self.Gway_Accel_Pedal_Position = None
        self.Gway_GearSelDisp = None
        self.Gway_Brake_Active = None
        self.Gway_Brake_Cylinder_Pressure = None
        
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0}

    def reset_trigger(self):
        self.reset = 1

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
            if self.acc_override or self.brk_override or self.steering_overide:
                print("OVERRIDE")
                # self.enable = 0
                # self.reset_trigger()

    def alive_counter(self, alv_cnt):
        alv_cnt += 1
        alv_cnt = 0 if alv_cnt > 255 else alv_cnt
        return alv_cnt
    
    def longitudinal_cmd(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.enable, 'PA_StrAngCmd': 0,
                   'LON_Enable': self.enable, 'Target_Brake': self.brake, 'Target_Accel': self.accel, 
                   'Alive_cnt': self.alv_cnt , 'Reset_Flag': self.reset}
        msg = self.db.encode_message('Control', signals)
        self.sender(0x210, msg)

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
            self.brk_override = res['Break_Override']
            self.steering_overide = res['Steering_Overide']
            self.safety_status = res['Safety_Status']
        if data.arbitration_id == 656:
            res = self.db.decode_message(656, data.data)
            self.temp_wheel = res['Gway_Steering_Angle']
        if self.timer(1):
            print("=================================================")
            # print("input acl:", self.accel, " | input brake:", self.brake)  
            # print("safety:", self.safety_status, " | brake_active:", self.Gway_Brake_Active)
            print("acc:", self.Gway_Accel_Pedal_Position, " | brk:", self.Gway_Brake_Cylinder_Pressure)
            print("ovr(acl,brk,str):", self.acc_override, "|", self.brk_override, "|", self.steering_overide," | reset:", self.reset)
            if self.enable:
                    print("ENABLE")
            else:
                print("DISABLE")
    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def controller(self):
        while 1:
            cmd = input('99: enable|88: disable|1001: reset\naccel:0~6|brake:-1~-20\n')
            cmd = int(cmd)
            if 0 <= cmd <= 6:
                self.accel = float(cmd)*5
                self.brake = 0
            elif -20 <= cmd <= -1:
                self.brake = -float(cmd)*5
                self.accel = 0
            elif cmd == 99: #enable
                self.enable = 1
                self.brake = 0
                self.accel = 0
                self.reset = 0
            elif cmd == 88: #disable
                self.enable = 0
                # self.reset_trigger()
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