#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import time

import rospy
from std_msgs.msg import Float32, Int8


class IoniqTransceiver():
    def __init__(self, CP):
        self.control_state = {
            'gear_en': 0x0,        # ON:0x1   OFF:0x0
            'steer_en': 0x0,       # ON:0x1   OFF:0x0
            'acc_en': 0x0,         # ON:0x1   OFF:0x0
        }
        self.CP = CP
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.target_steer = 0.
        self.temp_steer = 0.
        self.reset = 0
        self.db = cantools.database.load_file(CP.dbc)

        self.pub_velocity = rospy.Publisher(
            '/mobinha/car/velocity', Float32, queue_size=1)
        self.pub_gear = rospy.Publisher(
            '/mobinha/car/gear', Int8, queue_size=1)
        self.pub_mode = rospy.Publisher(
            '/mobinha/car/mode', Int8, queue_size=1)
        rospy.Subscriber(
            '/mobinha/control/wheel_angle', Float32, self.wheel_angle_cmd)
        rospy.Subscriber(
            '/mobinha/control/accel_brake', Float32, self.accel_brake_cmd)
        rospy.Subscriber(
            '/mobinha/planning/target_v', Float32, self.target_v_cb)

        self.accel_val = 0
        self.brake_val = 0
        self.wheel_angle = 0
        self.rcv_velocity = 0
        # self.reference_velocity = CP.maxEnableSpeed / 3.6
        self.target_v = 0
        self.tick = {0.01: 0, 0.02: 0, 0.2: 0, 0.5: 0, 0.09: 0}
        self.wheel = {'enable': False, 'current': 0, 'busy': False, 'step': 8}

        rospy.on_shutdown(self.cleanup)

    def target_v_cb(self, msg):
        self.target_v = msg.data

    def can_cmd(self, canCmd):
        state = self.control_state
        if canCmd.disable:  # Full disable
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
            self.reset = 1
        elif canCmd.enable:  # Full
            state = {**state, 'steer_en': 0x1, 'acc_en': 0x1}
            self.reset = 1
        elif canCmd.latActive:  # Only Lateral
            state = {**state, 'steer_en': 0x1, 'acc_en': 0x0}
            self.reset = 1
        elif canCmd.longActive:  # Only Longitudinal
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x1}
            self.reset = 1
        self.control_state = state

    def wheel_angle_cmd(self, msg):
        self.target_steer = msg.data        
        #TODO: lateral pid 
        # steer_dt = self.temp_steer-self.target_
        # for i in range(5):
            
    def accel_brake_cmd(self, msg):  # pid output is m/s^2 -10 ~ 10
        val_data = max(-4, min(4, msg.data))
        gain = 3
        if val_data > 0.:
            self.accel_val = val_data * gain
            self.brake_val = 0.0
        elif val_data <= 0.:
            self.accel_val = 0.0
            self.brake_val = val_data * -gain
            if self.target_v == 0.0 and self.rcv_velocity < 1:
                self.brake_val = 20.

    def receiver(self):
        data = self.bus.recv()
        try:
            if (data.arbitration_id == 0x130):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.indiLat = '%.4f' % res['Gway_Lateral_Accel_Speed']
                self.rcv_accel_value = '%.4f' % res['Gway_Longitudinal_Accel_Speed']
            if (data.arbitration_id == 0x280):
                res = self.db.decode_message(data.arbitration_id, data.data)
                # use
                self.velocity_FR = res['Gway_Wheel_Velocity_FR']
                # use
                self.velocity_RL = res['Gway_Wheel_Velocity_RL']
                # use
                self.velocity_RR = res['Gway_Wheel_Velocity_RR']
                # use
                self.velocity_FL = res['Gway_Wheel_Velocity_FL']
                self.rcv_velocity = (self.velocity_RR + self.velocity_RL)/7.2
                self.pub_velocity.publish(Float32(self.rcv_velocity))
            if (data.arbitration_id == 0x170):
                res = self.db.decode_message(data.arbitration_id, data.data)
                gear_sel_disp = res['Gway_GearSelDisp']                 # use
                if gear_sel_disp == 7:  # R
                    gear_sel_disp = 1
                elif gear_sel_disp == 6:  # N
                    gear_sel_disp = 2
                elif gear_sel_disp == 5:  # D
                    gear_sel_disp = 3
                else:  # P
                    gear_sel_disp = 0
                self.pub_gear.publish(Int8(gear_sel_disp))
            if (data.arbitration_id == 0x210):
                res = self.db.decode_message(data.arbitration_id, data.data)
                mode = 0
                if res['PA_Enable'] and res['LON_Enable']:
                    mode = 1
                self.pub_mode.publish(Int8(mode))
            if (data.arbitration_id == 641):
                res = self.db.decode_message(data.arbitration_id, data.data)
                plsRR = res['WHL_PlsRRVal']
                # print(plsRR)
            
            if(data.arbitration_id == 656):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.temp_steer = res['Gway_Steering_Angle']

        except Exception as e:
            print(e)

    def ioniq_control(self):
        signals = {'PA_Enable': self.control_state['steer_en'], 'PA_StrAngCmd': self.target_steer*self.CP.steerRatio,
                   'LON_Enable': self.control_state['acc_en'], 'Target_Brake': self.brake_val, 'Target_Accel': self.accel_val, 'Alive_cnt': 0x0, 'Reset_Flag': self.reset}
        msg = self.db.encode_message('Control', signals)
        self.sender(0x210, msg)
        self.reset = 0

    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False

    def cleanup(self):
        self.bus.shutdown()

    def run(self, CM):
        self.can_cmd(CM.CC.canCmd)
        if self.timer(0.02):
            self.ioniq_control()
        self.receiver()
