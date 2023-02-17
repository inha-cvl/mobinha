#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools

import math
import signal
import sys

import matplotlib.pyplot as plt

import rospy
from novatel_oem7_msgs.msg import INSPVA

import pymap3d
import time

import pandas as pd
import os

import threading

REAR_TREAD = 1.647 #  1,647mm
WHEEL_DIAMETER = 0.4826 + 0.12925*2# 19 * 0.0254 + tire thick * 2 
ENCODER_RESOLUTION = 46
KMH2MS = 1/3.6
FRAME_RATE = 50
WHEEL_BASE = 3
STEERRATIO = 17

class Activate_Signal_Interrupt_Handler:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('\nYou pressed Ctrl+C! Never use Ctrl+Z!')
        sys.exit(0)

class DistanceMeasurment:
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitreate=500000)
        self.db = cantools.database.load_file('../dbc/ioniq/can.dbc')

        self.pose_x = 0
        self.pose_y = 0
        self.pose_th = 0

        self.d = 0

        self.pls_RL = 0
        self.pls_RR = 0
        
        self.str_ang = 0
        
        self.encoder_cnt_L = 0
        self.encoder_cnt_R = 0

        self.prev_pls_RL = -1
        self.prev_pls_RR = -1

        self.posTh = 0
        self.posX = 0
        self.posY = 0

        self.diff_enc_cnt_L = 0
        self.diff_enc_cnt_R = 0

    def encoder_measure(self):
        while True:
            try:
                CAN_data = self.bus.recv(0)
                if CAN_data != None:
                    if CAN_data.arbitration_id == 640:
                        data = self.db.decode_message(CAN_data.arbitration_id, CAN_data.data)
                        v_RL = data['Gway_Wheel_Velocity_RL'] * KMH2MS
                        v_RL = data['Gway_Wheel_Velocity_RR'] * KMH2MS
                    if CAN_data.arbitration_id == 656:
                        data = self.db.decode_message(CAN_data.arbitration_id, CAN_data.data)
                        self.str_ang = data['Gway_Steering_Angle']
                    if CAN_data.arbitration_id == 641:
                        data = self.db.decode_message(CAN_data.arbitration_id, CAN_data.data)
                        if self.prev_pls_RL == -1 or self.prev_pls_RR == -1:
                            self.prev_pls_RL = data['WHL_PlsRLVal']
                            self.prev_pls_RR = data['WHL_PlsRRVal']
                        self.pls_RL = data['WHL_PlsRLVal']
                        self.pls_RR = data['WHL_PlsRRVal']

                        if self.prev_pls_RL != -1 and self.prev_pls_RR != -1:
                            self.diff_enc_cnt_L = self.pls_RL - self.prev_pls_RL
                            self.diff_enc_cnt_R = self.pls_RR - self.prev_pls_RR
                            self.diff_enc_cnt_L = self.diff_enc_cnt_L + 128 if self.diff_enc_cnt_L < 0 else self.diff_enc_cnt_L
                            self.diff_enc_cnt_R = self.diff_enc_cnt_R + 128 if self.diff_enc_cnt_R < 0 else self.diff_enc_cnt_R      

                            dL = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * self.diff_enc_cnt_L
                            dR = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * self.diff_enc_cnt_R
                            
                            dist = (dR + dL) / 2

                            dth = math.atan((dR - dL) / REAR_TREAD) #+ math.radians(self.str_ang / 13.6) # left + right -
                            dx = dist*math.cos(dth)
                            dy = dist*math.sin(dth)
                            dd=math.sqrt(dx**2 + dy**2)

                            lin_v = dist * FRAME_RATE
                            ang_v = lin_v * math.tan(math.radians(self.str_ang/STEERRATIO)) / WHEEL_BASE

                            sinTheta = math.sin(self.posTh)
                            cosTheta = math.cos(self.posTh)

                            if ang_v==0:
                                self.posX = self.posX + lin_v/FRAME_RATE * cosTheta
                                self.posY = self.posY + lin_v/FRAME_RATE * sinTheta
                            else:
                                velbyom = lin_v / ang_v
                                self.posTh = self.posTh + ang_v / FRAME_RATE
                                self.posX = self.posX + velbyom * (math.sin(self.posTh) - sinTheta)
                                self.posY = self.posY - velbyom * (math.cos(self.posTh) - cosTheta)

                            # plt.scatter(self.posX, self.posY, color = 'green', s=10, alpha=0.5)

                            self.pose_x = dx*math.cos(self.pose_th) - dy*math.sin(self.pose_th) + self.pose_x
                            self.pose_y = dx*math.sin(self.pose_th) + dy*math.cos(self.pose_th) + self.pose_y
                            self.pose_th += dth

                            # plt.scatter(self.pose_x, self.pose_y, color = 'red', s=10, alpha=0.5)
                            # plt.pause(0.02)

                            self.prev_pls_RL = self.pls_RL
                            self.prev_pls_RR = self.pls_RR

            except KeyboardInterrupt:
                        exit(0)

class GPSPlot:
    def __init__(self):
        rospy.Subscriber('/novatel/oem7/inspva',INSPVA, self.novatel_cb)
        
        self.lat = 0
        self.lon = 0

        self.init_lat = 0
        self.init_lon = 0
        self.init_yaw = 10000

        self.x = 0
        self.y = 0
        self.yaw = 0

    def novatel_cb(self, msg):
        if self.init_lat != 0:
            self.lat = msg.latitude
        else:
            self.init_lat = msg.latitude

        if self.init_lon != 0:
            self.lon = msg.longitude
        else:
            self.init_lon = msg.longitude
            
        if self.init_yaw != 10000:
            self.yaw = 90 - msg.azimuth if (msg.azimuth >= -90 and msg.azimuth <= 180) else -270 - msg.azimuth
        else:
            self.init_yaw = 90 - msg.azimuth if (msg.azimuth >= -90 and msg.azimuth <= 180) else -270 - msg.azimuth

    def gps_plot(self):
        while True:
            self.x, self.y, _ = pymap3d.geodetic2enu(self.lat, self.lon, 0, self.init_lat, self.init_lon, 0)
            self.r_x = math.cos(math.radians(self.init_yaw))*self.x - math.sin(math.radians(self.init_yaw))*self.y
            self.r_y = math.sin(math.radians(self.init_yaw))*self.x + math.cos(math.radians(self.init_yaw))*self.y
        # plt.scatter(self.x, self.y, color = 'blue', s=10, alpha=0.5)
        # plt.pause(0.02)


if __name__ == '__main__':
    Activate_Signal_Interrupt_Handler()
    rospy.init_node('gps_plot')

    GP = GPSPlot()
    DM = DistanceMeasurment()

    time.sleep(5)
    print(GP.init_yaw, GP.init_lat, GP.init_lon, GP.yaw, GP.lat, GP.lon)
    
    t1 = threading.Thread(target=DM.encoder_measure)
    t2 = threading.Thread(target=GP.gps_plot)
    t1.start()
    t2.start()  
    
    while True:
    # while not rospy.is_shutdown():
        data = {'time':[time.time()], 'lat':[GP.lat], 'lon':[GP.lon], 'enu_x':[GP.x], 'enu_y':[GP.y], 'yaw':[GP.yaw], 'r_x':[GP.r_x], 'r_y':[GP.r_y], 'init_lat':[GP.init_lat], 'init_lon':[GP.init_lon], 'init_yaw':[GP.init_yaw], 
        '2wheel_x':[DM.pose_x], '2wheel_y':[DM.pose_y], 'bicycle_x':[DM.posX], 'bicycle_y':[DM.posY], 'pulse_L':[DM.diff_enc_cnt_L], 'pulse_R':[DM.diff_enc_cnt_R], 'avg_tire_ang':[DM.str_ang/STEERRATIO]}
        df = pd.DataFrame(data)

        if not os.path.exists('./4.csv'):
            df.to_csv('./4.csv', index=False, mode='w', header=True)
        else:
            df.to_csv('./4.csv', index=False, mode='a', header=False)

    # plt.show()
    rospy.spin()
    
    
    




