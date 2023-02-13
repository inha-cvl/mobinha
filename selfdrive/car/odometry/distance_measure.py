#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
# import time
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pandas.core.indexes import interval

FRONT_TREAD = 1.638 # 1,638mm
REAR_TREAD = 1.647 #  1,647mm
WHEEL_DIAMETER = 0.4826 + 0.10 * 2 # 19 * 0.0254 + tire thick * 2 
ENCODER_RESOLUTION = 46
KMH2MS = 1 / 3.6

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

    def encoder_measure(self):
        prev_pls_RL = 0
        prev_pls_RR = 0
        
        pose_x = 0
        pose_y = 0
        pose_th = 0

        d = 0
        encoder_cnt_L = 0
        encoder_cnt_R = 0
        
        while True:
            try:
                CAN_data = self.bus.recv(0)
                if CAN_data != None:
                    if CCAN_data.arbitration_id == 656:
                        data = self.db.decode_message(CAN_data.arbitration_id, CAN_data.data)
                        str_ang = data['Gway_Steering_Angle']
                    if CCAN_data.arbitration_id == 641:
                        data = self.db.decode_message(CAN_data.arbitration_id, CAN_data.data)
                        pls_FL = data['WHL_PlsFLVal']
                        pls_FR = data['WHL_PlsFRVal']
                        pls_RL = data['WHL_PlsRLVal']
                        pls_RR = data['WHL_PlsRRVal']
                
                diff_enc_cnt_L = prev_pls_RL - pls_RL
                diff_enc_cnt_R = prev_pls_RR - pls_RR

                diff_enc_cnt_L = diff_enc_cnt_L + 128 if diff_enc_cnt_L < 0 else diff_enc_cnt_L
                diff_enc_cnt_R = diff_enc_cnt_R + 128 if diff_enc_cnt_R < 0 else diff_enc_cnt_R      

                dL = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * diff_enc_cnt_L
                dR = math.pi * WHEEL_DIAMETER / ENCODER_RESOLUTION * diff_enc_cnt_R
                
                dist = (dR + dL) / 2
                # str_ang = str_ang can signal
                dth = math.atan((dR - dL) / REAR_TREAD) + math.radians(str_ang / 13.73) # here is plus alpha ackerman  //// left + right -

                dx = dist*math.cos(dth)
                dy = dist*math.sin(dth)

                pose_x = dx*math.cos(pose_th) - dy*math.sin(pose_th) + pose_x
                pose_y = dx*math.sin(pose_th) + dy*math.cos(pose_th) + pose_y
                pose_th += dth

                prev_pls_RL = pls_RL
                prev_pls_RR = pls_RR

                dd=math.sqrt(dx**2 + dy**2)
                d+=dd
                if d // 10 == 0:
                    print(d," m")

                encoder_cnt_L += diff_enc_cnt_L
                encoder_cnt_R += diff_enc_cnt_R

                if 100 < d < 101:
                    print("encoder cnt_L: ", encoder_cnt_L)
                    print("encoder cnt_R: ", encoder_cnt_R)

                plt.scatter(pose_x, pose_y, color = 'red')
                plt.pause(0.1)

            except KeyboardInterrupt:
                        exit(0)
            
            plt.show()

if __name__ == '__main__':
    Activate_Signal_Interrupt_Handler()

    DM = DistanceMeasurment()
    DM.encoder_measure()
    