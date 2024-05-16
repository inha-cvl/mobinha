import can
import cantools
import threading
import time
from libs.pid import PID, APID
from libs.purepursuit import PurePursuit
from datetime import datetime
import matplotlib.pyplot as plt
import time
import numpy as np
from std_msgs.msg import Float32, Int8
from novatel_oem7_msgs.msg import INSPVA
import rospy
import pymap3d
import signal
import sys

class IONIQ:
    def __init__(self):
        rospy.init_node("testLateral")

        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/inha/Documents/catkin_ws/src/mobinha/selfdrive/car/dbc/ioniq/can.dbc')
        self.accel = 0
        self.brake = 0
        self.steer = 0

        self.LON_enable = 0
        self.PA_enable = 0
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

        geo_path = []
        with open('path_log.txt', 'r') as file:
            lines = file.readlines() 

        for line in lines:
            stripped_line = line.strip()
            lat, long = stripped_line.split(',')
            geo_path.append((float(lat), float(long)))
        self.base_lat = geo_path[0][0]
        self.base_lon = geo_path[0][1]

        self.path = []
        for i in range(len(geo_path)):
            x, y, _ = pymap3d.geodetic2enu(
            geo_path[i][0], geo_path[i][1], 0, self.base_lat, self.base_lon, 0)
            self.path.append((x,y))
        
        self.prev_steer = 0

        self.purepursuit = PurePursuit(self.path)
        
        # Plot 관련 초기화
        self.plot_lock = threading.Lock()
        self.time_stamps = []
        self.current_v_history = []
        self.target_v_history = []
        self.error_history = []
        self.run_time = time.time()

        # sensor data subscribe
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)
        self.x, self.y, self.z = 0, 0, 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0 

        self.idx = 0

        self.position = (0, 0)

    def reset_trigger(self):
        self.reset = 1

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False
        
    def daemon(self):
        while not rospy.is_shutdown():
            self.longitudinal_cmd() # update alive count, send CAN msg
            self.longitudinal_rcv() # receive CAN msg, print out current values
            if self.acc_override or self.brk_override: 
                self.LON_enable = 0
            if self.steering_overide:
                self.PA_enable = 0

    def alive_counter(self, alv_cnt):
        alv_cnt += 1
        alv_cnt = 0 if alv_cnt > 255 else alv_cnt
        return alv_cnt
    
    def longitudinal_cmd(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.PA_enable, 'PA_StrAngCmd': self.steer,
                   'LON_Enable': self.LON_enable, 'Target_Brake': self.brake, 'Target_Accel': self.accel, 
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
        if (data.arbitration_id == 529):
            res = self.db.decode_message(data.arbitration_id, data.data)
            self.PA_Enable_Status = res['PA_Enable_Status']
            self.LON_Enable_Status = res['LON_Enable_Status']
        if self.timer(1):
            print(f"=================================================\n \
                input acl: {self.accel} | input brake: {self.brake}\n  \
                safety: {self.safety_status} | brake_active: {self.Gway_Brake_Active}\n  \
                acc: {self.Gway_Accel_Pedal_Position} | brk: {self.Gway_Brake_Cylinder_Pressure}\n  \
                ovr(acl,brk,str): {self.acc_override} | {self.brk_override} | {self.steering_overide}| reset: {self.reset}\n \
                accel_pedal: {self.Gway_Accel_Pedal_Position}, brake_cylinder:{self.Gway_Brake_Cylinder_Pressure} \
                LON_en: {self.LON_enable}, PA_en: {self.PA_enable}")

    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def state_controller(self):
        while not rospy.is_shutdown():
            cmd = input('99: PA |88: LON |77: ALL\n \
                        1001: reset\n1000: over\n')
            cmd = int(cmd)
            if cmd == 99: 
                self.PA_enable = 1
                self.LON_enable = 0
                self.brake = 0
                self.accel = 0
                self.reset = 0
            elif cmd == 88: 
                self.PA_enable = 0
                self.LON_enable = 1
                self.brake = 0
                self.accel = 0
                self.reset = 0
            elif cmd == 77: 
                self.PA_enable = 1
                self.LON_enable = 1
                self.brake = 0
                self.accel = 0
                self.reset = 0
            elif cmd == 1001:
                self.reset_trigger()
            elif cmd == 1000:
                exit(0)

    def controller(self):
        while not rospy.is_shutdown():
            if self.LON_enable:
                self.accel, self.brake = self.apid.run(self.current_v, self.target_v)   
                time.sleep(0.01)

            self.position = (self.x, self.y)
            cte = self.calculate_cte(self.position)
            if self.PA_enable:
                
                wheel_angle, (lx, ly) = self.purepursuit.run(self.current_v, self.path, self.position, self.yaw, cte)
                threshold = 449
                self.steer = int(self.limit_steer_change(min(max(int(wheel_angle*13.5), -threshold), threshold)))
                print(self.steer)
                if abs(self.steer - self.prev_steer) > 100:
                    print(f"v : {self.prev_current_v:.2f}, pos: {self.prev_position[0]:.2f}. {self.prev_position[1]:.2f}, heading: {self.prev_yaw:.2f}, target: {lx:.2f}, {ly:.2f}, idx: {self.prev_idx}")
                    print(f"v : {self.current_v:.2f}, pos: {self.position[0]:.2f}. {self.position[1]:.2f}, heading: {self.yaw:.2f}, target: {lx:.2f}, {ly:.2f}, idx: {self.idx}")
                    exit(0)
                self.prev_current_v = self.current_v
                self.prev_position = self.position
                self.prev_yaw = self.yaw
                self.prev_steer = self.steer
                self.prev_idx = self.idx
                # print(self.steer)
                time.sleep(0.01)

                ## v : 1.8489583333333333, pos: (-20.275103792422193, 27.635642078066144), heading: -144.01293900697044, target: 0.11273516760735534, -0.5387912950040479

    
    def update_values(self):
        with self.plot_lock:
            current_time = time.time() - self.run_time
            self.time_stamps.append(current_time)
            self.current_v_history.append(self.current_v*3.6)
            self.target_v_history.append(self.target_v*3.6)
            self.error_history.append(abs(self.target_v - self.current_v)*3.6)

            for arr in [self.time_stamps, self.current_v_history, self.target_v_history, self.error_history]:
                if len(arr) > 100:
                    arr.pop(0)
    
    def set_target_v(self):
        while not rospy.is_shutdown():
            self.target_v = 5 / 3.6

    def plot_velocity(self):
        plt.ion()
        fig, ax = plt.subplots()
        target_line, = ax.plot(self.time_stamps, self.target_v_history, label='Target Velocity')
        current_line, = ax.plot(self.time_stamps, self.current_v_history, label='Current Velocity')
        error_line, = ax.plot(self.time_stamps, self.error_history, label='Error')
        plt.legend(loc='upper left')

        while not rospy.is_shutdown():
            self.update_values()

            target_line.set_ydata(self.target_v_history)
            target_line.set_xdata(self.time_stamps)

            current_line.set_ydata(self.current_v_history)
            current_line.set_xdata(self.time_stamps)

            error_line.set_ydata(self.error_history)
            error_line.set_xdata(self.time_stamps)

            ax.relim()
            ax.autoscale_view()

            plt.grid(True)
            plt.draw()
            plt.pause(0.01)
    
    def plot_position(self):
        plt.ion()
        fig, ax = plt.subplots()
        path_line, = ax.plot([el[0] for el in self.path], [el[1] for el in self.path], "r", label="path")
        position_point, = ax.plot([], [], 'bo', label="Current Position")
        plt.legend(loc='upper left')
        plt.grid(True)

        while not rospy.is_shutdown():
            with self.plot_lock:
                position_point.set_xdata(self.x)
                position_point.set_ydata(self.y)
                ax.relim()
                ax.autoscale_view()

                fig.canvas.draw_idle()
                fig.canvas.flush_events()

            plt.pause(0.01)
    
    
    def novatel_cb(self, msg):
        self.x, self.y, self.z = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, 0, self.base_lat, self.base_lon, 0)
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = 90 - msg.azimuth + 360 if (-270 <= 90 - msg.azimuth <= -180) else 90 - msg.azimuth
        # print(self.yaw)


    def calc_idx(self, pt):
        min_dist = float('inf')
        min_idx = 0

        for idx, pt1 in enumerate(self.path):
            dist = np.sqrt((pt[0]-pt1[0])**2+(pt[1]-pt1[1])**2)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx

        if min_idx == len(self.path) - 1:
            pt1 = self.path[min_idx-1]
        else:
            pt1 = self.path[min_idx]

        return min_idx
    
    def calculate_cte(self, position):
        idx = self.calc_idx(position)
        self.idx = idx
        Ax, Ay = self.path[idx]
        Bx, By = self.path[idx+1]
        Px, Py = position

        numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
        denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
        cte = numerator / denominator if denominator != 0 else 0

        cross_product = (Bx - Ax) * (Py - Ay) - (By - Ay) * (Px - Ax)
        
        if cross_product > 0:
            return -cte
        elif cross_product < 0:
            return cte
        else:
            return 0
    
    def limit_steer_change(self, current_steer):
        return current_steer
    
def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Exiting gracefully...')
    rospy.signal_shutdown('Exiting')
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    IONIQ = IONIQ()
    t1 = threading.Thread(target=IONIQ.daemon)
    t2 = threading.Thread(target=IONIQ.state_controller)
    t3 = threading.Thread(target=IONIQ.set_target_v)
    t4 = threading.Thread(target=IONIQ.controller)
    # t5 = threading.Thread(target=IONIQ.plot_velocity)
    t5 = threading.Thread(target=IONIQ.plot_position)

    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()

    t1.join()
    t2.join()
    t3.join()
    t4.join()
    # t5.join()
