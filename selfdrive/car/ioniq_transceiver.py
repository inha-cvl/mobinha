#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import time

import rospy
from std_msgs.msg import Float32, Int8, Int8MultiArray, MultiArrayLayout, MultiArrayDimension, Float64
from geometry_msgs.msg import Vector3

import threading

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
        #rospy.Subscriber( '/mobinha/control/target_actuators', Vector3, self.target_actuators_cb)
        self.gateway = Int8MultiArray()
        #gatway info / 0: PA_Enable, 1: LON_Enable, 2: Accel_Override, 3: Break_Override, 4: Steering_Overide, 5: Reset_Flag
        self.gateway.data = [0, 0, 0, 0, 0, 0]

        # self.lock = threading.Lock()
        self.rcv_velocity = 0
        self.tick = {0.01: 0, 0.02: 0, 0.2: 0, 0.5: 0, 0.09: 0, 1: 0}
        self.Accel_Override = 0
        self.Break_Override = 0
        self.Steering_Overide = 0
        self.alv_cnt = 0
        self.Alive_Count_ERR = 0
        self.recv_err_cnt = 0
        self.prev_control_state = self.control_state.copy()

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
        self.mode = 0
        if canCmd.enable:
            self.mode = 1
        if self.Accel_Override or self.Break_Override or self.Steering_Overide or self.Alive_Count_ERR:
            self.mode = 2 # override mode
            state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
            self.reset_trigger()
        
        self.control_state = state 
        self.pub_mode.publish(Int8(self.mode))

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
        data = self.bus.recv(0.02)
        if data is None:
            self.recv_err_cnt +=1
            print("recv err cnt: ", self.recv_err_cnt)
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
                if self.Accel_Override or self.Break_Override or self.Steering_Overide:
                    self.target_actuators['steer'] = self.ego_actuators['steer']
                    self.target_actuators['accel'] = self.ego_actuators['accel']
                    self.target_actuators['brake'] = self.ego_actuators['brake']
        except Exception as e:
            print(e)

    def alive_counter(self, alv_cnt):
        return (alv_cnt + 1) % 256
    
    def ioniq_control(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.control_state['steer_en'], 'PA_StrAngCmd': self.target_actuators['steer'],
                   'LON_Enable': self.control_state['acc_en'], 'Target_Brake': self.target_actuators['brake'], 
                   'Target_Accel': self.target_actuators['accel'], 'Alive_cnt': self.alv_cnt, 'Reset_Flag': self.reset}
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

    def run_not_thread(self, CM):
        self.can_cmd(CM.CC.canCmd)
        self.set_actuators(CM.CC.actuators)
        if self.timer(0.02):
            # if self.prev_control_state != self.control_state:
            #     self.init_target_actuator()
            #     self.prev_control_state = self.control_state.copy()
            self.ioniq_control()
            self.pub_gateway.publish(self.gateway)
            self.pub_gateway_time.publish(Float64(time.time()))
        self.receiver()

    def run(self, CM):
        # 병렬로 실행될 수 있도록 메서드를 수정하거나 추가
        control_thread = threading.Thread(target=self.control_task, args=(CM,))
        receiver_thread = threading.Thread(target=self.receiver_task)
        
        control_thread.start()
        receiver_thread.start()
        
        control_thread.join()
        receiver_thread.join()

    def control_task(self, CM):
        # with self.lock: # Aquire lock
        self.can_cmd(CM.CC.canCmd)
        self.set_actuators(CM.CC.actuators)
        self.ioniq_control()
        self.pub_gateway.publish(self.gateway)
        self.pub_gateway_time.publish(Float64(time.time()))

    def receiver_task(self):
        # with self.lock: # Acquire lock
        self.receiver()
        