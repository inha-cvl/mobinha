#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import time
import datetime
import math
import rospy
from std_msgs.msg import Float32, Int8, Int8MultiArray, Int16MultiArray, MultiArrayLayout, MultiArrayDimension, Float64
from geometry_msgs.msg import Vector3

class IoniqTransceiver():
    def __init__(self, CP):
        self.control_state = {
            'steer_en': 0x0,       # ON:0x1   OFF:0x0
            'acc_en': 0x0,         # ON:0x1   OFF:0x0
        }
        self.ego_actuators = {'steer': 0,'accel': 0,'brake': 0}
        self.target_actuators = {'steer': 0,'accel': 0,'brake': 0}

        self.CP = CP
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.mode = 0
        self.reset = 0
        self.db = cantools.database.load_file(CP.dbc)

        self.pub_velocity = rospy.Publisher('/mobinha/car/velocity', Float32, queue_size=1)
        self.pub_gear = rospy.Publisher('/mobinha/car/gear', Int8, queue_size=1)
        self.pub_mode = rospy.Publisher('/mobinha/car/mode', Int8, queue_size=1)
        self.pub_ego_actuators = rospy.Publisher('/mobinha/car/ego_actuators', Vector3, queue_size=1)
        self.pub_gateway = rospy.Publisher('/mobinha/car/gateway', Int8MultiArray, queue_size=1)
        self.pub_gateway_time = rospy.Publisher('/mobinha/car/gateway_time', Float64, queue_size=1)
        self.pub_rpm = rospy.Publisher('/mobinha/car/rpm', Float32, queue_size=1)
        rospy.Subscriber('/mobinha/planning/blinker', Int8, self.blinker_cb)
        rospy.Subscriber('/sensor_check', Int16MultiArray, self.sensor_check_cb)
        #rospy.Subscriber( '/mobinha/control/target_actuators', Vector3, self.target_actuators_cb)

        #gatway info / 0: PA_Enable, 1: LON_Enable, 2: Accel_Override, 3: Break_Override, 4: Steering_Overide, 5: Reset_Flag
        self.gateway = Int8MultiArray()
        self.gateway.data = [0, 0, 0, 0, 0, 0]
        self.rcv_velocity = 0
        self.tick = {0.01: 0, 0.02: 0, 0.2: 0, 0.5: 0, 0.09: 0, 1: 0}
        self.Accel_Override = 0
        self.Break_Override = 0
        self.Steering_Overide = 0
        self.alv_cnt = 0
        self.Alive_Count_ERR = 0
        self.recv_err_cnt = 0
        self.err_time = None 
        self.last_mode_2_time = 0
        self.force_mode_2 = False
        self.blinker = {'left':0, 'right': 0}
        self.PA_Enable_Status = 0
        self.LON_Enable_Status = 0
        self.prev_control_state = self.control_state.copy()
        self.sensor_state_list = [0, 0, 0, 0, 0, 0, 0]

        rospy.on_shutdown(self.cleanup)

    def reset_trigger(self):
        if self.Accel_Override or self.Break_Override or self.Steering_Overide:
            self.reset = 1
        elif self.reset and (self.Accel_Override and self.Break_Override and self.Steering_Overide)== 0:
            self.reset = 0

    def can_cmd(self, canCmd):
        state = self.control_state
        
        if canCmd.disable:  # Full disable
            self.reset_trigger()
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
        elif canCmd.enable:  # Full
            state = {**state, 'steer_en': 0x1, 'acc_en': 0x1}
        elif canCmd.latActive:  # Only Lateral
            state = {**state, 'steer_en': 0x1, 'acc_en': 0x0}
        elif canCmd.longActive:  # Only Longitudinal
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x1}

        if self.Accel_Override or self.Break_Override or self.Steering_Overide:
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
            self.reset_trigger()
        
        if 0 in self.sensor_state_list:
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
            # self.reset_trigger()
        
        self.control_state = state 
    
    def check_mode(self):
        current_time = rospy.get_time()
        if self.force_mode_2 and current_time - self.last_mode_2_time < 2.0:
            # mode를 강제로 2로 2초 동안 유지
            self.mode = 2
        else:
            self.force_mode_2 = False  # 강제 유지를 종료
            if self.PA_Enable_Status and self.LON_Enable_Status:
                self.mode = 1
            else:
                self.mode = 0
            
            if self.Accel_Override or self.Break_Override or self.Steering_Overide:
                self.mode = 2  # override mode
                self.last_mode_2_time = current_time  # 현재 시간을 저장
                self.force_mode_2 = True  # mode를 강제로 2로 유지
            
        if 0 in self.sensor_state_list:
            self.mode = 3

        self.pub_mode.publish(Int8(self.mode))

    def blinker_cb(self, msg):
        if msg.data == 1: # left blinker
            self.blinker = {**self.blinker, 'left':1, 'right':0}
        elif msg.data == 2: # right blinker
            self.blinker = {**self.blinker, 'left':0, 'right':1}
        else:
            self.blinker = {**self.blinker, 'left':0, 'right':0}
    
    def sensor_check_cb(self, msg):
        #sensor_check.data = [cam.check(), lidar.check(), gps.check(), ins.check(), can.check(), perception.check(), planning.check()]
        self.sensor_state_list = msg.data # 1 normal, 0 error


    def target_actuators_cb(self, msg):
        self.target_actuators['steer'] = msg.x
        self.target_actuators['accel'] = msg.y
        self.target_actuators['brake'] = msg.z

    def init_target_actuator(self):
        self.target_actuators['steer'] = 0
        self.target_actuators['accel'] = 0
        self.target_actuators['brake'] = 0
        
    def set_actuators(self, actuators):
        if self.mode == 1:
            self.target_actuators['steer'] = actuators.steer
            self.target_actuators['accel'] = actuators.accel
            self.target_actuators['brake'] = actuators.brake
        else:
            self.init_target_actuator()

    def receiver(self):
        data = self.bus.recv(0.2)
        try:
            if (data.arbitration_id == 304):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.ego_actuators['brake'] = res['Gway_Brake_Cylinder_Pressure']
                
            if (data.arbitration_id == 0x280):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.velocity_FR = res['Gway_Wheel_Velocity_FR']
                self.velocity_RL = res['Gway_Wheel_Velocity_RL']
                self.velocity_RR = res['Gway_Wheel_Velocity_RR']
                self.velocity_FL = res['Gway_Wheel_Velocity_FL']
                self.rcv_velocity = (self.velocity_RR + self.velocity_RL)/7.2
                self.pub_velocity.publish(Float32(self.rcv_velocity))
                wheel_diameter = 0.4826 + 0.12925*2
                rpm = self.rcv_velocity/3.6*60 / (math.pi*wheel_diameter)
                self.pub_rpm.publish(Float32(rpm))

            if (data.arbitration_id == 368):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.ego_actuators['accel'] = res['Gway_Accel_Pedal_Position']
                gear_sel_disp = res['Gway_GearSelDisp']
                if gear_sel_disp == "R":  # R
                    gear_sel_disp = 1
                elif gear_sel_disp == "N":  # N
                    gear_sel_disp = 2
                elif gear_sel_disp == "D":  # D
                    gear_sel_disp = 3
                else:  # P
                    gear_sel_disp = 0
                self.pub_gear.publish(Int8(gear_sel_disp))
            if(data.arbitration_id == 656):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.ego_actuators['steer'] = res['Gway_Steering_Angle']
                vector3 = Vector3()
                vector3.x = self.ego_actuators['steer']
                vector3.y = self.ego_actuators['accel']
                vector3.z = self.ego_actuators['brake']
                self.pub_ego_actuators.publish(vector3)
            if (data.arbitration_id == 784):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.Accel_Override = res['Accel_Override']
                self.Break_Override = res['Break_Override']
                self.Steering_Overide = res['Steering_Overide']
                self.Alive_Count_ERR = res['Alive_Count_ERR']
                self.gateway.data[2] = res['Accel_Override']
                self.gateway.data[3] = res['Break_Override']
                self.gateway.data[4] = res['Steering_Overide']
            if (data.arbitration_id == 529):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.PA_Enable_Status = res['PA_Enable_Status']
                self.LON_Enable_Status = res['LON_Enable_Status']

            if self.err_time is not None:
                recovery_time = datetime.datetime.now() - self.err_time
                print(f"Recovered in {recovery_time.total_seconds()} seconds")
                self.err_time = None
            
        except Exception as e:
            if data is None:
                self.recv_err_cnt += 1
                print("recv err cnt: ", self.recv_err_cnt)
                if self.err_time is None:
                    self.err_time = datetime.datetime.now()
            else:
                print(e)

    def alive_counter(self, alv_cnt):
        return (alv_cnt + 1) % 256
    
    def ioniq_control(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.control_state['steer_en'], 'PA_StrAngCmd': self.target_actuators['steer'],
                   'LON_Enable': self.control_state['acc_en'], 'Target_Brake': self.target_actuators['brake'], 
                   'Target_Accel': self.target_actuators['accel'], 'Alive_cnt': self.alv_cnt, 'Reset_Flag': self.reset,
                   'TURN_SIG_LEFT': self.blinker['left'], 'TURN_SIG_RIGHT':self.blinker['right']}
        self.gateway.data[0] = signals['PA_Enable']
        self.gateway.data[1] = signals['LON_Enable']
        self.gateway.data[5] = signals['Reset_Flag']
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

    def run(self, CM):
        if self.timer(0.02):
            self.can_cmd(CM.CC.canCmd)
            self.check_mode()
            self.set_actuators(CM.CC.actuators)
            self.ioniq_control()
            self.pub_gateway.publish(self.gateway)
            self.pub_gateway_time.publish(Float64(time.time()))
        self.receiver()
        # print(self.PA_Enable_Status, self.LON_Enable_Status)
        