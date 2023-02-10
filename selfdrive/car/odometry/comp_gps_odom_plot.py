#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
# import time

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

        except KeyboardInterrupt:
                    exit(0)

if __name__ == '__main__':
    Activate_Signal_Interrupt_Handler()

    DM = DistanceMeasurment()
    DM.encoder_measure()