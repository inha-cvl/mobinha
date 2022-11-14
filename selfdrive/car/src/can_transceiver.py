#!/usr/bin/python3
#sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import threading
import time
import binascii
import os

import rospy
import rospkg
from std_msgs.msg import Int16, Float32, Float32MultiArray, String

class CanTransceiver():
    def __init__(self):
        self.control_state = {
                'manual' : 0x0,         # ON:0x1   OFF:0x0
                'mode_a' : 0x0,         # ON:0x1   OFF:0x0
                'mode_b' : 0x0,         # ON:0x1   OFF:0x0
                'mode_c' : 0x0,         # ON:0x1   OFF:0x0
                'gear_en' : 0x0,        # ON:0x1   OFF:0x0                
                'steer_en' : 0x3,       # ON:0x5   OFF:0x3
                'acc_en' : 0x0,         # ON:0x1   OFF:0x0
                'acc_init' : False      # For slight delay Mode A -> C
                }

        #CAN Init
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitreate=500000)
        self.db = cantools.database.load_file('/home/inha/catkin_ws/src/niro/control/src/dbc/niro_2021.dbc')
        self.msg_id = {'w_BRK11': 0x200, 'w_BRK12': 0x500, 'w_BRK21': 0x220, 'w_BRK22': 0x230, 'w_BRK23': 0x380, 'w_BRK24': 0x240}

        #ROS Init
        rospy.init_node('can_transceiver', anonymous=False)
        self.pub_velocity = rospy.Publisher('/car_v', Float32, queue_size=1)
        self.sub_can_cmd = rospy.Subscriber('/can_cmd', Int16, self.can_cmd)
        self.sub_wheel_angle = rospy.Subscriber('/wheel_angle', Float32, self.wheel_angle_cmd)
        self.sub_accel_brake = rospy.Subscriber('/accel_brake', Float32, self.accel_brake_cmd)
        # self.sub_gear = rospy.Subscriber('/gear', String, self.gear_cmd)
        self.sub_velocity = rospy.Subscriber('/velocity', Float32, self.velocity_cmd)
        
        self.accel_value = 0
        self.wheel_angle = 0
        self.gear = 'N'

        self.counter = {'w_BRK11': 0, 'w_BRK12': 0, 'w_BRK21': 0, 'w_BRK22': 0, 'w_BRK23': 0, 'w_BRK24': 0}
        self.tick = {0.01: 0, 0.02: 0, 0.2: 0, 0.5: 0, 0.09 : 0}
        self.wheel = {'enable' : False, 'current' : 0, 'busy': False, 'step' : 3}

        rospy.on_shutdown(self.cleanup)
#######  ROS-Subscriber  #####################################################
    def can_cmd(self, msg):
        data = msg.data
        state = self.control_state
        if data == 0: # Full disable
            state = {**state, 'manual_control' : 0x0, 'steer_en' : 0x3, 'mode_a' : 0x0}
            state['acc_init'] = False
        elif data == 1: # Only Lateral
            state = {**state, 'steer_en' : 0x5, 'manual_control' : 0x1}
            state['acc_init'] = False
        elif data == 2: # Mode AB -> Mid
            state = {**state, 'manual' : 0x1, 'mode_a' : 0x1, 'mode_b' : 0x1, 'mode_c' : 0x0}
        elif data ==3: # Full
            state = {**state, 'manual' : 0x1, 'mode_a' : 0x1, 'mode_b' : 0x1, 'mode_c' : 0x1, 'acc_en' : 0x01}
        elif data ==4: # Only Longi
            state = {**state, 'steer_en' : 0x3, 'manual' : 0x1, 'mode_a' : 0x1, 'mode_b' : 0x1, 'mode_c' : 0x1, 'acc_en' : 0x01}


        self.control_state = state

        # self.manual_control = data[0]
        # self.mode_a = data[1]
        # self.mode_b = data[2]
        # self.mode_c = data[3]
        # self.gear_en = data[4]
        # self.steer_en = data[5]
        # self.vel_mode = data[6]

    def wheel_angle_cmd(self, msg):
        wheel = self.wheel 
        step = wheel['step']
        angle = int(msg.data * 10)
        if not wheel['busy']:
            wheel['busy'] = True
            if wheel['current'] > angle:
                step *= -1
            for i in range(wheel['current'], angle, step):
                self.steer(i/10)
                time.sleep(0.013)
            self.steer(angle/10)

            wheel['current'] = angle
            wheel['busy'] = False
            self.wheel = wheel
        else:
            return

    def accel_brake_cmd(self, msg):
        if msg.data >= 1.5: # Was 5.0
            self.accel_value = 1.5 
        elif msg.data <= -10.0:
            self.accel_value = -10.0
        else:
            self.accel_value = msg.data

    # def gear_cmd(self, msg):
        # self.gear_mode = msg.data

    def velocity_cmd(self, msg):
        self.reference_velocity = msg.data

##############################################################################
    def transmitter(self):
        if self.timer(0.01):
            self.set_gear(self.gear)
        if self.timer(0.02):
            self.accelerator('w_BRK11')
            self.accelerator('w_BRK21')
            self.accelerator('w_BRK22')
            self.accelerator('w_BRK24')
        if self.timer(0.2):
            self.accelerator('w_BRK23')
        if self.timer(0.5):
            self.accelerator('w_BRK12')
        self.receiver()

    def receiver(self):
        data = self.bus.recv()
        try:
            if (data.arbitration_id == 0x130):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.indiLat = '%.4f' % res['Gway_Lateral_Accel_Speed']
                self.rcv_accel_value = '%.4f' % res['Gway_Longitudinal_Accel_Speed']
            elif (data.arbitration_id == 0x280):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.velocity_FR = res['Gway_Wheel_Velocity_FR']                 # use
                self.velocity_RL = res['Gway_Wheel_Velocity_RL']                 # use
                self.velocity_RR = res['Gway_Wheel_Velocity_RR']                 # use
                self.velocity_FL = res['Gway_Wheel_Velocity_FL']                 # use
                self.rcv_velocity = (self.velocity_RR + self.velocity_RL)/7.2 # when it is revise tell to all
                # print(self.rcv_velocity)
                self.pub_velocity.publish(self.rcv_velocity)
            elif (data.arbitration_id == 0x290):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.rcv_wheel_angle = res['Gway_Steering_Angle']     # use
                self.rcv_steer_status = res['Gway_Steering_Status']                   # use
            elif (data.arbitration_id == 0x260):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.rcv_break = res['Gway_Brake_Active'] # not use
            elif (data.arbitration_id == 0x170):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.rcv_gear = res['Gway_GearSelDisp']                 # use

        except Exception as e:
            print(e)

    def steer(self, angle):
        msg = self.db.encode_message('PA', {'PA_Enable': self.control_state['steer_en'], 'PA_StrAngCmd': angle * 13.73}) #13.73
        self.sender(0x210, msg)

    def accelerator(self, msg_name):
        msg_id = self.msg_id[msg_name]
        self.counter[msg_name] = (lambda x: ( x + 1) % 15)(self.counter[msg_name])

        if msg_name == 'w_BRK11':
            signals = {'brk11_35_0': 0x0, 'AliveCounter11': self.counter[msg_name], 'ChkSum11': 0}

        elif msg_name == 'w_BRK12':
            signals = {'brk12_4_0': 0x0}

        elif msg_name == 'w_BRK21':
            signals = {'brk21_Mode_A': self.control_state['mode_a'], 'brk21_3_1': 0x1, 'AliveCounter21': self.counter[msg_name]
                    , 'brk21_8_8': 0x32, 'brk21_1_16': 0x1, 'brk21_2_17': 0x0
                    , 'brk21_3_19': 0x4, 'brk21_2_22': 0x1, 'brk21_9_24': 0xC8
                    , 'brk21_11_33': 0x1E, 'brk21_12_44': 0x6A4, 'brk21_8_56': 0x0}

        elif msg_name == 'w_BRK22':
            signals = {'brk22_I': self.control_state['manual'], 'brk22_Mode_B': self.control_state['mode_b'], 'brk22_1_15': 0x0
                    , 'brk22_aReq1': self.accel_value, 'brk22_1_35': 0x0, 'brk22_aReq2': self.accel_value
                    , 'AliveCounter22': self.counter[msg_name], 'ChkSum22': 0}
            
        elif msg_name == 'w_BRK23':
            signals = {'brk23_3_0': 0x0, 'brk23_1_3': 0x1, 'brk23_2_12': 0x0}
                        
        elif msg_name == 'w_BRK24':
            signals = {'brk24_6_0': 0xF, 'brk24_6_6': 0xF, 'brk24_7_12': 0x7F
                    , 'brk24_7_19': 0x7F, 'brk24_Mode_C': self.control_state['mode_c'], 'brk24_3_39': 0x0
                    , 'AliveCounter24': self.counter[msg_name], 'ChkSum24': 0, 'brk24_8_56': 0x0}

        msg = self.db.encode_message(msg_name, signals)
        msg = self.checksum(msg, msg_name)
        self.sender(msg_id, msg)
    
    def set_gear(self, gear='N'):  # choice mode in P, R, N and D
        self.gear = gear
        signals = {
                'SFT_P': 0x2, 'SFT_R': 0x2, 'SFT_N': 0x2, 'SFT_D': 0x2,
                'SFT_P_Reversed': 0x1, 'SFT_R_Reversed': 0x1, 'SFT_N_Reversed': 0x1, 'SFT_D_Reversed': 0x1,
                'SFT_CTL_EN': self.control_state['gear_en']
                }

        if gear == "P":
            signals['SFT_P'] = 0x1
            signals['SFT_P_Reversed'] = 0x2
        elif gear == "R":
            signals['SFT_R'] = 0x1
            signals['SFT_R_Reversed'] = 0x2
        elif gear == "N":
            signals['SFT_N'] = 0x1
            signals['SFT_N_Reversed'] = 0x2
        elif gear == "D":
            signals['SFT_D'] = 0x1
            signals['SFT_D_Reversed'] = 0x2

        msg = self.db.encode_message('w_SFT', signals)
        self.sender(0x110, msg)

    def checksum(self, msg, msg_name):
        if msg_name == ('w_BRK11' or 'w_BRK22'):
            temp = list(msg.hex())
            p1 = int(temp[0], 16) + int(temp[2], 16) + int(temp[4], 16) + int(temp[6], 16) + int(temp[8], 16) + int(temp[10], 16) + int(temp[12], 16) + int(temp[14], 16)
            p2 = int(temp[1], 16) + int(temp[3], 16) + int(temp[5], 16) + int(temp[7], 16) + int(temp[9], 16) + int(temp[11], 16) + int(temp[13], 16) + int(temp[15], 16)
            res = (0x10 - ((p1 + p2) & 0x0f) & 0b1111)
            temp[14] = format(res, 'x')
            temp = ''.join(temp)
            return bytes.fromhex(temp)

        elif msg_name == 'w_BRK24':
            temp = list(msg.hex())
            p1 = int(temp[0], 16) + int(temp[2], 16) + int(temp[4], 16) + int(temp[6], 16) + int(temp[8], 16) + int(temp[10], 16) + int(temp[12], 16) + int(temp[14], 16)
            p2 = int(temp[1], 16) + int(temp[3], 16) + int(temp[5], 16) + int(temp[7], 16) + int(temp[9], 16) + int(temp[11], 16) + int(temp[13], 16) + int(temp[15], 16)
            res = (0x10 - ((p1 + p2) & 0x0f) & 0b1111)
            temp[12] = format(res, 'x')
            temp = ''.join(temp)
            return bytes.fromhex(temp)

        else:
            return msg

    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id, data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False

    def cleanup(self):
        self.bus.shutdown()
        
if __name__ == '__main__': 
    cantransceiver = CanTransceiver()
    while 1:
        cantransceiver.transmitter()
        # cantransceiver.receiver()
