import can
import cantools
import threading
import time
from libs.pid import PID, APID
from datetime import datetime

class IONIQ:
    def __init__(self):
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/inha/Documents/catkin_ws/src/mobinha/selfdrive/car/dbc/ioniq/can.dbc')
        self.accel = 0
        self.brake = 0
        self.steer = 0

        self.enable = 0
        self.alv_cnt = 0
        self.reset = 0
        self.time = time.time()

        self.acc_override = None
        self.brk_override = None
        self.steering_overide = None
        self.safety_status = None

        self.Gway_Steering_Angle = 0
        self.Gway_Accel_Pedal_Position = None
        self.Gway_GearSelDisp = None
        self.Gway_Brake_Active = None
        self.Gway_Brake_Cylinder_Pressure = None
        
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0}
        self.target_v = 0
        self.current_v = 0
        self.apid = APID()

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
            self.longitudinal_cmd() # update alive count, send CAN msg
            self.longitudinal_rcv() # receive CAN msg, print out current values
            if self.acc_override or self.brk_override or self.steering_overide:
                print("OVERRIDE")
                self.enable = 0
                # self.reset_trigger()

    def alive_counter(self, alv_cnt):
        alv_cnt += 1
        alv_cnt = 0 if alv_cnt > 255 else alv_cnt
        return alv_cnt
    
    def longitudinal_cmd(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.enable, 'PA_StrAngCmd': self.steer,
                   'LON_Enable': self.enable, 'Target_Brake': self.brake, 'Target_Accel': self.accel, 
                   'Alive_cnt': self.alv_cnt , 'Reset_Flag': self.reset,
                   'TURN_SIG_LEFT': 0, 'TURN_SIG_RIGHT': 0
                   }
        msg = self.db.encode_message('Control', signals)
        self.sender(0x210, msg)

    def longitudinal_rcv(self):
        data = self.bus.recv()
        if data.arbitration_id == 0x280:
            res = self.db.decode_message(0x280, data.data)
            self.velocity_FR = res['Gway_Wheel_Velocity_FR']
            self.velocity_RL = res['Gway_Wheel_Velocity_RL']
            self.velocity_RR = res['Gway_Wheel_Velocity_RR']
            self.velocity_FL = res['Gway_Wheel_Velocity_FL']
            self.current_v = (self.velocity_RR + self.velocity_RL)/7.2
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
            self.Gway_Steering_Angle = res['Gway_Steering_Angle']
        if self.timer(1):
            print(f"=================================================\n+ \
                  input acl: {self.accel} | input brake: {self.brake}\n + \
                  safety: {self.safety_status} | brake_active: {self.Gway_Brake_Active}\n + \
                  acc: {self.Gway_Accel_Pedal_Position} | brk: {self.Gway_Brake_Cylinder_Pressure}\n + \
                  ovr(acl,brk,str): {self.acc_override} | {self.brk_override} | {self.steering_overide}| reset: {self.reset}\n")
            if self.enable:
                print("ENABLE")
            else:
                print("DISABLE")

    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def state_controller(self):
        while 1:
            cmd = input('99: enable|88: disable|\n \
                        1001: reset\n1000: over\n')
            cmd = int(cmd)
            if cmd == 99: #enable
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
    
    def set_target_v(self):
        ## manual
        ## case 1 : constant
        self.target_v = 20 / 3.6 # km/h

        ## case 2 : sinusoidal
        # amplitude = 5
        # offset = 20
        # number = datetime.datetime.now().microsecond%1000000
        # while number >= 10:
        #     number //= 10
        # self.target_v = offset + amplitude * np.sin(number*2*np.pi/9) / 3.6

        ## case 3 : step
        # amplitude = 5
        # offset = 20
        # number = datetime.datetime.now().second%10
        # if number < 5:
        #     step = 1
        # else:
        #     step = -1
        # self.target_v = offset + amplitude*step / 3.6

    def long_controller(self):
        self.accel, self.brake = self.apid.run(self.target_v, self.current_v)

if __name__ == '__main__':
    IONIQ = IONIQ()
    t1 = threading.Thread(target=IONIQ.daemon)
    t2 = threading.Thread(target=IONIQ.state_controller)
    t3 = threading.Thread(target=IONIQ.set_target_v)
    t4 = threading.Thread(target=IONIQ.long_controller)

    t1.start()
    t2.start()
    t3.start()
    t4.start()

    t1.join()
    t2.join()
    t3.join()
    t4.join()