#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import time

import rospy
from std_msgs.msg import Float32, Int16


class IoniqTransceiver():
    def __init__(self, CP):
        self.control_state = {
            'gear_en': 0x0,        # ON:0x1   OFF:0x0
            'steer_en': 0x0,       # ON:0x1   OFF:0x0
            'acc_en': 0x0,         # ON:0x1   OFF:0x0
        }
        self.CP = CP
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitreate=500000)
        self.steer_angle = 0.
        self.msg_id = {'w_BRK11': 0x200, 'w_BRK12': 0x500, 'w_BRK21': 0x220,
                       'w_BRK22': 0x230, 'w_BRK23': 0x380, 'w_BRK24': 0x240}
        self.db = cantools.database.load_file(CP.dbc)

        self.pub_velocity = rospy.Publisher(
            '/mobinha/car/velocity', Float32, queue_size=1)
        rospy.Subscriber('/mobinha/visualize/can_cmd', Int16, self.can_cmd)
        rospy.Subscriber(
            '/mobinha/control/wheel_angle', Float32, self.wheel_angle_cmd)
        rospy.Subscriber(
            '/mobinha/control/accel_brake', Float32, self.accel_brake_cmd)

        self.accel_value = 0
        self.accel_val = 0
        self.brake_val = 0
        self.wheel_angle = 0
        self.gear = 'N'

        self.counter = {'w_BRK11': 0, 'w_BRK12': 0,
                        'w_BRK21': 0, 'w_BRK22': 0, 'w_BRK23': 0, 'w_BRK24': 0}
        self.tick = {0.01: 0, 0.02: 0, 0.2: 0, 0.5: 0, 0.09: 0}
        self.wheel = {'enable': False, 'current': 0, 'busy': False, 'step': 8}

        rospy.on_shutdown(self.cleanup)

    def can_cmd(self, msg):
        data = msg.data
        state = self.control_state
        if data == 0:  # Full disable
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
        elif data == 1:  # Only Lateral
            state = {**state, 'steer_en': 0x1}
        elif data == 2:  # Mode AB -> Mid
            state = {**state, 'acc_en': 0x0}
        elif data == 3:  # Full
            state = {**state, 'acc_en': 0x1}
        elif data == 4:  # Only Longitudinal
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x1}
        self.control_state = state

    def wheel_angle_cmd(self, msg):
        steer = self.wheel
        self.wheel_angle = int(msg.data) * 10
        if not steer['busy']:
            steer['busy'] = True
            if steer['current'] < self.wheel_angle:
                for i in range(int(steer['current']), self.wheel_angle, steer['step']):
                    self.steer_angle == i/10
                    time.sleep(0.05)
                    steer['current'] = i
            else:
                for i in range(int(steer['current']), self.wheel_angle, -steer['step']):
                    self.steer_angle == i/10
                    time.sleep(0.05)
                    steer['current'] = i
            steer['busy'] = False
            self.wheel = steer
        else:
            return

    def accel_brake_cmd(self, msg):
        val_data = max(-10, min(10, msg.data))
        if val_data > 0.:
            self.accel_val = val_data * 10.0
            self.brake_val = 0.0
        elif val_data <= 0.:
            self.accel_val = 0.0
            self.brake_val = val_data * -10.0
            if self.reference_velocity == 0.0 and self.rcv_velocity < 3.5:
                self.brake_val = 100.

    def receiver(self):
        data = self.bus.recv()
        try:
            if (data.arbitration_id == 0x130):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.indiLat = '%.4f' % res['Gway_Lateral_Accel_Speed']
                self.rcv_accel_value = '%.4f' % res['Gway_Longitudinal_Accel_Speed']
            elif (data.arbitration_id == 0x280):
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
                self.pub_velocity.publish(self.rcv_velocity)
            elif (data.arbitration_id == 0x290):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.rcv_wheel_angle = res['Gway_Steering_Angle']     # use
                self.rcv_steer_status = res['Gway_Steering_Status']
            elif (data.arbitration_id == 0x170):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.rcv_gear = res['Gway_GearSelDisp']                 # use

        except Exception as e:
            print(e)

    def steer(self, angle):
        msg = self.db.encode_message(
            'PA', {'PA_Enable': self.control_state['steer_en'], 'PA_StrAngCmd': angle * self.CP.steerRatio})  # 13.73
        self.sender(0x210, msg)

    def ioniq_control(self):
        signals = {'PA_Enable': self.control_state['steer_en'], 'PA_StrAngCmd': self.steer_angle * self.CP.steerRatio,
                   'LON_Enable': self.control_state['acc_en'], 'Target_Brake': self.brake_val, 'Target_Accel': self.accel_val, 'Alive_cnt': 0x0}
        msg = self.db.encode_message('Control', signals)
        self.sender(0x210, msg)

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

    def run(self):
        if self.timer(0.02):
            self.ioniq_control()
        self.receiver()
