#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools

import math
import signal
import sys

import matplotlib.pyplot as plt


FRONT_TREAD = 1.638 # 1,638mm
REAR_TREAD = 1.647 #  1,647mm
WHEEL_DIAMETER = 0.4826 + 12925*2# 19 * 0.0254 + tire thick * 2 
ENCODER_RESOLUTION = 46

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

    def encoder_measure(self):
        while True:
            try:
                CAN_data = self.bus.recv(0)
                if CAN_data != None:
                    if CAN_data.arbitration_id == 656:
                        data = self.db.decode_message(CAN_data.arbitration_id, CAN_data.data)
                        self.str_ang = data['Gway_Steering_Angle']
                    if CAN_data.arbitration_id == 641:
                        data = self.db.decode_message(CAN_data.arbitration_id, CAN_data.data)
                        if self.prev_pls_RL == -1 or self.prev_pls_RR == -1:
                            self.prev_pls_RL = data['WHL_PlsRLVal']
                            self.prev_pls_RR = data['WHL_PlsRRVal']
                        pls_FL = data['WHL_PlsFLVal']
                        pls_FR = data['WHL_PlsFRVal']
                        self.pls_RL = data['WHL_PlsRLVal']
                        self.pls_RR = data['WHL_PlsRRVal']

                        if self.prev_pls_RL != -1 and self.prev_pls_RR != -1:
                            diff_enc_cnt_L = self.pls_RL - self.prev_pls_RL
                            diff_enc_cnt_R = self.pls_RR - self.prev_pls_RR
                            diff_enc_cnt_L = diff_enc_cnt_L + 128 if diff_enc_cnt_L < 0 else diff_enc_cnt_L
                            diff_enc_cnt_R = diff_enc_cnt_R + 128 if diff_enc_cnt_R < 0 else diff_enc_cnt_R      

                            dL = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * diff_enc_cnt_L
                            dR = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * diff_enc_cnt_R
                            
                            dist = (dR + dL) / 2

                            dth = math.atan((dR - dL) / REAR_TREAD) #+ math.radians(self.str_ang / 13.6) # left + right -
                            dx = dist*math.cos(dth)
                            dy = dist*math.sin(dth)
                            dd=math.sqrt(dx**2 + dy**2)

                            self.pose_x = dx*math.cos(self.pose_th) - dy*math.sin(self.pose_th) + self.pose_x
                            self.pose_y = dx*math.sin(self.pose_th) + dy*math.cos(self.pose_th) + self.pose_y
                            self.pose_th += dth

                            self.d+=dd

                            self.encoder_cnt_L += diff_enc_cnt_L
                            self.encoder_cnt_R += diff_enc_cnt_R

                            if 100 < self.d < 101:
                                print(self.d," m")
                                print("encoder cnt_L: ", self.encoder_cnt_L)
                                print("encoder cnt_R: ", self.encoder_cnt_R)

                            self.prev_pls_RL = self.pls_RL
                            self.prev_pls_RR = self.pls_RR

                            plt.scatter(self.pose_x, self.pose_y, color = 'red', alpha=0.5)
                            plt.pause(0.01)

            except KeyboardInterrupt:
                        exit(0)
            
        plt.show()

if __name__ == '__main__':
    Activate_Signal_Interrupt_Handler()

    DM = DistanceMeasurment()
    DM.encoder_measure()