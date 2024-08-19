import can
import cantools
import threading
import time
from libs.apid import APID
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
import libs.utils as utils
from jsk_recognition_msgs.msg import BoundingBoxArray

class IONIQ:
    def __init__(self):
        rospy.init_node("controlTest_junmyeong")
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/inha/Documents/catkin_ws/src/mobinha/selfdrive/car/dbc/ioniq/can.dbc')
        self.accel = 0
        self.brake = 0
        self.steer = 0
        self.vApid_output = 0
        self.sApid_output = 0

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
        
        self.target_v = 0
        self.current_v = 0
        self.vApid = APID()
        self.sApid = APID()
        
        # global path init
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
        # print(self.path)
        # plt.plot([el[0] for el in self.path], [el[1] for el in self.path])
        # plt.show()

        # controller init
        self.prev_steer = 0
        self.lx = 0
        self.ly = 0
        self.cte = 0

        self.purepursuit = PurePursuit(self.path)
        
        # Plot init
        self.plot_lock = threading.Lock()
        self.time_stamps = []
        self.current_v_history = []
        self.target_v_history = []
        self.error_history = []
        self.accel_history = [0]
        self.vApid_output_history = []
        self.sApid_output_history = []
        self.final_output_history = []
        self.target_s_history = []
        self.current_s_history = []
        self.obs_velocity_history = []
        self.run_time = time.time()


        # sensor data subscribe
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)
        rospy.Subscriber('/target_velocity', Float32, self.target_v_cb)
        rospy.Subscriber('/current_distance', Float32, self.current_s_manual_cb)
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.current_s_cb)
        self.x, self.y, self.z = 0, 0, 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.current_s = 100
        self.target_s = 30
        self.output = 0
        self.idx = 0

        self.position = (0, 0)

        self.reset_trigger()

        # current v publish
        self.pub_vel = rospy.Publisher('/current_velocity', Float32, queue_size=1)


        self.obs_velocity = None
        self.last_box_distance = None

    def current_s_cb(self, msg):
        min_distance = 100
        obs_velocity = 100
        for box in msg.boxes:
            box_distance = np.sqrt(box.pose.position.x**2 + box.pose.position.y**2)
            if box_distance < min_distance:
                min_distance = box_distance
                obs_velocity = box.value
        
        self.current_s = min_distance

        # self.obs_velocity = obs_velocity
        try:
            self.obs_velocity = (min_distance - self.last_box_distance) / 0.1 * 3.6
        except:
            pass
        self.last_box_distance = min_distance

    def novatel_cb(self, msg):
        self.x, self.y, self.z = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, 0, self.base_lat, self.base_lon, 0)
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = 90 - msg.azimuth + 360 if (-270 <= 90 - msg.azimuth <= -180) else 90 - msg.azimuth

    def target_v_cb(self, msg):
        self.target_v = float(msg)

    def current_s_manual_cb(self, msg):
        self.current_s = msg.data

    def daemon(self):
        while not rospy.is_shutdown():
            self.longitudinal_cmd() 
            self.longitudinal_rcv() 
            
            # long
            if self.acc_override or self.brk_override: 
                self.LON_enable = 0
            
            # lat
            if self.steering_overide:
                self.PA_enable = 0

            # all
            # if self.acc_override or self.brk_override or self.steering_overide:
            #     self.PA_enable = 0
            #     self.LON_enable = 0 


    def set_target_v(self):
        # self.target_v = 50/3.6
        while not rospy.is_shutdown():
            safety_distance = max(self.current_v*3.6-15, 10) # safe_distance
            margin = 3
            margined_safety_distance = safety_distance + margin
            
            if self.current_s < margined_safety_distance*0.9:
                status = "danger_zone"
            elif margined_safety_distance*0.8 < self.current_s < margined_safety_distance*1.2:
                status = "safe_zone"
            elif margined_safety_distance*1.2 < self.current_s:
                status = "far_zone"
            else:
                status = "Error"
            
            print(f"safe distance {margined_safety_distance}, cur distance {self.current_s}, {status}")

            if status == "danger_zone":
                self.target_v = 0/3.6
                # self.obs_velocity - beta
            elif status == "safe_zone":
                self.target_v = 10/3.6
                # self.obs_velocity
            elif status == "far_zone":
                self.target_v = 15/3.6
                # min(self.target_v, self.obs_velocity + alpha)
            else:
                print("error on status decision: test.py")

            # self.target_v /= 3.6
            # self.target_v = 50/3.6
            # pass
            # while not rospy.is_shutdown():
            #     timeflow_sec = int(time.time())-int(self.run_time)
            #     if timeflow_sec < 5:
            #         self.target_v = 0
            #     else:
            #         self.target_v = 20/3.6
            #     # elif timeflow_sec < 10:
            #     #     self.target_v = 30 / 3.6

            #     # elif timeflow_sec < 20:
            #     #     self.target_v = 30 / 3.6

            #     # elif timeflow_sec < 30:
            #     #     self.target_v = 30 / 3.6

            #     # elif timeflow_sec < 40:
            #     #     self.target_v = 30 / 3.6

            #     # elif timeflow_sec < 50:
            #     #     self.target_v = 30 / 3.6

            #     # elif timeflow_sec < 60:
            #     #     self.target_v = 0 / 3.6

    def reset_trigger(self):
        self.reset = 1



    def longitudinal_cmd(self):
        self.alv_cnt = utils.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.PA_enable, 'PA_StrAngCmd': self.steer,
                   'LON_Enable': self.LON_enable, 'Target_Brake': self.brake, 'Target_Accel': self.accel, 
                   'Alive_cnt': self.alv_cnt, 'Reset_Flag': self.reset,
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
            self.current_v = (self.velocity_RR + self.velocity_RL)/7.2 # [m/s]
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
        if utils.timer(1):
            print(f"=================================================\n \
                input acl: {self.accel} | input brake: {self.brake}\n  \
                safety: {self.safety_status} | brake_active: {self.Gway_Brake_Active}\n  \
                acc: {self.Gway_Accel_Pedal_Position} | brk: {self.Gway_Brake_Cylinder_Pressure}\n  \
                ovr(acl,brk,str): {self.acc_override} | {self.brk_override} | {self.steering_overide}| reset: {self.reset}\n \
                accel_pedal: {self.Gway_Accel_Pedal_Position}, brake_cylinder:{self.Gway_Brake_Cylinder_Pressure} \
                LON_en: {self.LON_enable}, PA_en: {self.PA_enable}")
        
        msg = Float32()
        msg.data = self.current_v
        self.pub_vel.publish(msg)


    def sender(self, arb_id, msg):
        can_msg = can.Message(arbitration_id=arb_id,
                              data=msg, is_extended_id=False)
        self.bus.send(can_msg)


    def state_controller(self):
        while not rospy.is_shutdown():
            try:
                cmd = input('99: PA |88: LON |77: ALL\n \
                            1001: reset\n1000: over\n')
                cmd = int(cmd)
                if cmd == 99: 
                    self.reset_trigger()
                    self.PA_enable = 1
                    self.LON_enable = 0
                    self.brake = 0
                    self.accel = 0
                    self.reset = 0
                elif cmd == 88:
                    self.reset_trigger()
                    self.PA_enable = 0
                    self.LON_enable = 1
                    self.brake = 0
                    self.accel = 0
                    self.reset = 0
                elif cmd == 77: 
                    self.reset_trigger()
                    self.PA_enable = 1
                    self.LON_enable = 1
                    self.brake = 0
                    self.accel = 0
                    self.reset = 0
                elif cmd == 1001:
                    self.reset_trigger()
                elif cmd == 1000:
                    exit(0)
            except:
                print("re-insert")


    def controller(self):
        while not rospy.is_shutdown():
            ### Longitudinal control
            if self.LON_enable:
                # # self.target_s = max(self.current_v*3.6-15, 5) # safe_distance
                # self.target_s = 15 # for sAPID test

                self.vApid_output = self.vApid.run(self.current_v, self.target_v)   
                # self.vApid_output = 100

                # self.sApid_output = -self.sApid.run(self.current_s, self.target_s)  
                # output = min(self.vApid_output, self.sApid_output)
                # self.output = output
                output = self.vApid_output
                self.output = output
                # ### post process ####
                accel_lim = 20
                brake_lim = 60
                # if abs(self.target_s - self.current_s) < 4:
                #     output *= abs(self.target_s - self.current_s)/4
                if output > 0:
                    self.accel = min(output, accel_lim)
                    self.brake = 0
                else:
                    self.accel = 0
                    self.brake = min(-output*1.2, brake_lim)
                
                if self.target_v == 0 and self.current_v < 2.5:
                    self.brake = 40
                # #####################
            
            ### Lateral control
            self.position = (self.x, self.y)
            self.idx, self.cte = utils.calculate_cte(self.path, self.position)
            if self.PA_enable:                
                wheel_angle, (self.lx, self.ly) = self.purepursuit.run(self.current_v, self.path[self.idx:], self.position, self.yaw, self.cte)
                threshold = 450
                self.steer = utils.limit_steer_change(self.current_v, self.prev_steer, min(max(wheel_angle*13.5, -threshold), threshold))
                # print("Steer: ", self.steer)
                inted_steer = int(self.steer)
                self.prev_current_v = self.current_v
                self.prev_position = self.position
                self.prev_yaw = self.yaw
                self.prev_steer = inted_steer
                self.prev_idx = self.idx
            time.sleep(0.01)


    def update_values(self):
        with self.plot_lock:
            current_time = time.time() - self.run_time
            self.time_stamps.append(current_time)
            self.current_v_history.append(self.current_v*3.6)
            self.target_v_history.append(self.target_v*3.6)
            self.error_history.append(abs(self.target_v - self.current_v)*3.6)
            try:
                self.accel_history.append((self.current_v_history[-1]-self.current_v_history[-2])*100)
            except:
                self.accel_history.append(0)
            self.vApid_output_history.append(self.vApid_output)
            self.sApid_output_history.append(self.sApid_output)
            self.final_output_history.append(self.output)
            # self.final_output_history.append(min(self.vApid_output, self.sApid_output))
            self.current_s_history.append(self.current_s)
            self.target_s_history.append(self.target_s)
            self.obs_velocity_history.append(self.obs_velocity)

            for arr in [self.time_stamps, self.current_v_history, self.target_v_history, self.error_history, self.vApid_output_history, self.sApid_output_history, self.final_output_history, self.target_s_history, self.current_s_history, self.obs_velocity_history]:
                if len(arr) > 200:
                    arr.pop(0)


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
    

    def plot_acceleration(self):
        plt.ion()
        fig, ax = plt.subplots()
        current_line, = ax.plot(self.time_stamps, self.accel_history, label='Current Acceleration')
        
        plt.legend(loc='upper left')

        while not rospy.is_shutdown():
            self.update_values()

            current_line.set_ydata(self.current_v_history)
            current_line.set_xdata(self.time_stamps)

            ax.relim()
            ax.autoscale_view()

            plt.grid(True)
            plt.draw()
            plt.pause(0.01)


    def plot_apids(self):
        plt.ion()
        fig, ax = plt.subplots()
        current_line1, = ax.plot(self.time_stamps, self.vApid_output_history, label='vApid output', c='b')
        current_line2, = ax.plot(self.time_stamps, self.sApid_output_history, label='sApid output', c='g')
        current_line3, = ax.plot(self.time_stamps, self.final_output_history, label='final output', c='r')
        
        plt.legend(loc='upper left')

        while not rospy.is_shutdown():
            self.update_values()

            current_line1.set_ydata(self.vApid_output_history)
            current_line1.set_xdata(self.time_stamps)
            current_line2.set_ydata(self.sApid_output_history)
            current_line2.set_xdata(self.time_stamps)
            current_line3.set_ydata(self.final_output_history)
            current_line3.set_xdata(self.time_stamps)

            ax.relim()
            ax.autoscale_view()

            plt.grid(True)
            plt.draw()
            plt.pause(0.01)

    def plot_distances(self):
        plt.ion()
        fig, ax = plt.subplots()
        current_line1, = ax.plot(self.time_stamps, self.target_s_history, label='target_s', c='b')
        current_line2, = ax.plot(self.time_stamps, self.current_s_history, label='curent_s', c='g')
        
        plt.legend(loc='upper left')

        while not rospy.is_shutdown():
            self.update_values()

            current_line1.set_ydata(self.target_s_history)
            current_line1.set_xdata(self.time_stamps)
            current_line2.set_ydata(self.current_s_history)
            current_line2.set_xdata(self.time_stamps)
            ax.relim()
            ax.autoscale_view()

            plt.ylim(0, 50)
            plt.grid(True)
            plt.draw()
            plt.pause(0.01)
    
    def plot_obs_velocty(self):
        plt.ion()
        fig, ax = plt.subplots()
        current_line1, = ax.plot(self.time_stamps, self.obs_velocity_history, label='obs_velocity', c='b')
        
        plt.legend(loc='upper left')

        while not rospy.is_shutdown():
            self.update_values()

            current_line1.set_ydata(self.obs_velocity_history)
            current_line1.set_xdata(self.time_stamps)
            ax.relim()
            ax.autoscale_view()

            plt.ylim(-5, 5)
            plt.grid(True)
            plt.draw()
            plt.pause(0.01)
            
    
    def plot_position(self):
        plt.ion()
        fig, ax = plt.subplots()
        path_line, = ax.plot([el[0] for el in self.path], [el[1] for el in self.path], "r", label="path")
        position_point, = ax.plot([], [], 'bo', label="Current Position")
        lookahead, = ax.plot([], [], 'ro', label="Lookahead Point")

        plt.legend(loc='upper left')
        plt.grid(True)
        plt.axis('equal')

        while not rospy.is_shutdown():
            with self.plot_lock:
                position_point.set_xdata(self.x)
                position_point.set_ydata(self.y)
                lookahead.set_xdata(self.lx)
                lookahead.set_ydata(self.ly)
                ax.relim()
                ax.autoscale_view()

                fig.canvas.draw_idle()
                fig.canvas.flush_events()

            plt.pause(0.01)
    

    

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Exiting gracefully...')
    rospy.signal_shutdown('Exiting')
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, utils.signal_handler)

    IONIQ = IONIQ()
    t1 = threading.Thread(target=IONIQ.daemon)
    t2 = threading.Thread(target=IONIQ.state_controller)
    t3 = threading.Thread(target=IONIQ.set_target_v)
    t4 = threading.Thread(target=IONIQ.controller)

    t5 = threading.Thread(target=IONIQ.plot_velocity)
    # t5 = threading.Thread(target=IONIQ.plot_position)
    # t5 = threading.Thread(target=IONIQ.plot_acceleration)
    # t5 = threading.Thread(target=IONIQ.plot_apids)
    # t5 = threading.Thread(target=IONIQ.plot_distances)
    # t5 = threading.Thread(target=IONIQ.plot_obs_velocty)

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
