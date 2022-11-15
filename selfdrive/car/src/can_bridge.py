#!/usr/bin/python3
from std_msgs.msg import Int8, Bool, Int16, Int8MultiArray, Int16MultiArray
import rospy
import threading
import signal
import time
import dbc.niro.cankey as cankey
import cantools
import can
import sys
import os
import json

python_package_path = os.path.expanduser(
    os.path.join('~', '.local/lib/python3.8/site-packages'))
sys.path.append(python_package_path)


class Activate_Signal_Interrupt_Handler:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('\nYou pressed Ctrl+C! Never use Ctrl+Z!')
        sys.exit(0)


'''
record:
[brake_pedal, accel_pedal, sas_angle, gear, rpm, vel]
switch:
[brake_pedal, accel_pedal, drvtq, e-stop, sensor error, system error, lane departure, AEB]
'''


class Bridge:
    def __init__(self):
        #######CAN############

        f = open("src/fingerprint.json")
        data = json.load(f)
        dbc_path = data['car_state']['dbc_file_path']
        wheel_size = float(data['control_param']['wheel_size'])
        f.close()

        self.db = cantools.database.load_file(
            dbc_path)
        self.CCAN = can.ThreadSafeBus(
            interface='kvaser', channel=0, bitrate=500000)
        self.SCC = can.ThreadSafeBus(
            interface='kvaser', channel=1, bitrate=500000)
        self.CCAN.set_filters(cankey.CCAN_filters)
        self.SCC.set_filters(cankey.SCC_filters)

        self.WHEELSIZE = wheel_size  # m
        self.bsd_array = [0, 0]
        self.rcv_wheel_speed = 0
        self.rpm = 0
        self.gear_map = {'P': 0, 'R': 1, 'N': 2, 'D': 3}
        self.mode = 0
        self.prev_mode = 0
        self.vel = 0
        self.prev_vel = 0

        self.e_ems11_data = {'Brake_Pedal_Pos': 0, 'Accel_Pedal_Pos': 0}
        self.elect_gear_data = {'Elect_Gear_Shifter': 'P'}
        self.scc11_data = {'MainMode_ACC': 0,
                           'VSetDis': 0, 'ACC_ObjDist': 0, 'TTC': 10e2}
        self.scc12_data = {'ACCMode': 0, 'CF_VSM_Warn': 0,
                           'aReqRaw': 0, 'aReqValue': 0}
        self.scc14_data = {'ComfortBandLower': 0, 'JerkLowerLimit': 0}
        self.sas11_data = {'SAS_Angle':  0}  # from SAS11
        self.mdps11_data = {'CR_Mdps_DrvTq': 0}
        self.clu_data = {'CF_Clu_Vanz': 0,
                         'CF_Clu_RheostatLevel': 0, 'CF_Clu_VanzDecimal': 0}
        self.lkas11_data = {'CF_Lkas_MsgCount': 0}
        self.fca11_data = {'CF_VSM_Warn': 0}
        self.obj_dist = 0
        self.last_obj_dist = 0  # from obj_dist

        self.prev_lkas_chksum = 0
        self.cur_lkas_chksum = 0

        self.aeb_time = time.time()

        ########ROS################
        rospy.init_node('Bridge_CAN', anonymous=False)

        rospy.Subscriber('/mode', Int8, self.mode_callback)
        rospy.Subscriber('/target_vel', Int8, self.vel_set_callback)
        rospy.Subscriber('/BSD_check', Int8MultiArray, self.BSD_callback)
        rospy.Subscriber('/op_fcw', Bool, self.op_fcw_callback)
        rospy.Subscriber('/curvature', Int16, self.curvature_callback)

        self.can_record = rospy.Publisher(
            '/can_record', Int16MultiArray, queue_size=1)
        self.can_switch = rospy.Publisher(
            '/can_switch', Int8MultiArray, queue_size=1)
        self.radar_state = rospy.Publisher('/radar', Bool, queue_size=1)
        self.aeb_state = rospy.Publisher('/aeb', Bool, queue_size=1)
        self.clu_cruise = rospy.Publisher('/clu_cruise', Int8, queue_size=1)

        self.can_record_data = Int16MultiArray()
        self.can_record_data.data = [0, 0, 0, 0, 0, 0]

        self.can_switch_data = Int8MultiArray()
        self.can_switch_data.data = [0, 0, 0]
        self.switch_cnt = 0
        self.switch_on = False

        self.radar = Bool()
        self.radar.data = False
        self.aeb = Bool()
        self.aeb.data = False
        self.op_fcw = False

        self.curvature = 0

        self.curv_avg = [500 for _ in range(15)]

    def mode_callback(self, msg):
        self.mode = msg.data

    def vel_set_callback(self, msg):
        self.vel = int(msg.data)

    def BSD_callback(self, msg):
        self.bsd_array = msg.data

    def op_fcw_callback(self, msg):
        self.op_fcw = msg.data

    def curvature_callback(self, msg):
        self.curvature = msg.data
        self.curv_avg.pop(0)
        self.curv_avg.append(min(self.curvature, 500))
        self.curvature = sum(self.curv_avg)/len(self.curv_avg)

    def set_SCC11(self, data):  # 1056
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.scc11_data['MainMode_ACC'] = data['MainMode_ACC']
        self.scc11_data['VSetDis'] = data['VSetDis']
        self.scc11_data['ACC_ObjDist'] = data['ACC_ObjDist']
        TTC = min(abs(data['ACC_ObjDist']/(data['ACC_ObjRelSpd']+0.1)), 100)
        self.scc11_data['TTC'] = abs(
            TTC) if data['ACC_ObjRelSpd'] < 0 else 10e1

    def set_SCC12(self, data):  # 1057
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.scc12_data['ACCMode'] = True if data['ACCMode'] == "enabled" else False
        self.scc12_data['CF_VSM_Warn'] = data['CF_VSM_Warn']
        can_msg = self.generate_brake(data)  # Create SCC12 force brake
        return can_msg  # Create SCC12 force brake

    def generate_brake(self, data):
        if self.rcv_wheel_speed > 10 and (self.scc11_data['TTC'] <= 2.5 or self.op_fcw):
            # Proportional gain for smooth(?) braking
            target_acc = self.rcv_wheel_speed * -0.3
            target_acc = max(min(target_acc, -3), -9)
            data['aReqRaw'] = target_acc
            data['aReqValue'] = target_acc
            data['CR_VSM_ChkSum'] = 0
            signal = self.db.encode_message('SCC12', data)
            checksum = (0x10 - sum(sum(divmod(i, 16)) for i in signal)) % 0x10
            data['CR_VSM_ChkSum'] = checksum
        msg = self.db.encode_message('SCC12', data)
        can_msg = can.Message(arbitration_id=1057,
                              data=msg, is_extended_id=False)
        return can_msg

    def set_SCC14(self, data):  # 1057
        data = self.db.decode_message(data.arbitration_id, data.data)
        can_msg = self.generate_jerk(data)
        return can_msg

    def generate_jerk(self, data):
        self.scc14_data['JerkLowerLimit'] = max(
            data['JerkLowerLimit'], self.scc14_data['JerkLowerLimit'])
        self.scc14_data['ComfortBandLower'] = max(
            data['ComfortBandLower'], self.scc14_data['ComfortBandLower'])
        if(self.scc11_data['TTC'] <= 2.5 or self.op_fcw):
            data['JerkLowerLimit'] = 10
        msg = self.db.encode_message('SCC14', data)
        can_msg = can.Message(arbitration_id=905,
                              data=msg, is_extended_id=False)
        return can_msg

    def set_SAS11(self, data):  # 688
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.sas11_data['SAS_Angle'] = data['SAS_Angle']

    def set_FCA11(self, data):
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.sas11_data['CF_VSM_Warn'] = data['CF_VSM_Warn']

    def set_MDPS11(self, data):  # 897
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.mdps11_data['CR_Mdps_DrvTq'] = data['CR_Mdps_DrvTq']

    def set_E_EMS11(self, data):  # 881
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.e_ems11_data['Brake_Pedal_Pos'] = data['Brake_Pedal_Pos']
        self.e_ems11_data['Accel_Pedal_Pos'] = data['Accel_Pedal_Pos']

    def set_WHL_SPD11(self, data):  # 902
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.rcv_wheel_speed = (data['WHL_SPD_RL'] + data['WHL_SPD_RR']) * 0.5
        self.rpm = self.rcv_wheel_speed / (0.1885*self.WHEELSIZE)

    def set_ELECT_GEAR(self, data):  # 882
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.elect_gear_data['Elect_Gear_Shifter'] = data['Elect_Gear_Shifter']

    def set_CLU11(self, data):  # 1265
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.clu_data['CF_Clu_Vanz'] = data['CF_Clu_Vanz']
        self.clu_data['CF_Clu_VanzDecimal'] = data['CF_Clu_VanzDecimal']
        self.clu_data['CF_Clu_RheostatLevel'] = data['CF_Clu_RheostatLevel']
        clu_cruise = data['CF_Clu_CruiseSwMain']
        self.clu_cruise.publish(clu_cruise)

    def set_LKAS11(self, data):  # 832
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.lkas11_data['CF_Lkas_Chksum'] = data['CF_Lkas_Chksum']

    # Thread 1
    def bridge(self):
        cur_time = time.time()
        bsd_time = time.time()
        while 1:
            try:
                # Send SCC to CCAN
                SCC_data = self.SCC.recv(0)
                if SCC_data != None:
                    SCC_id = SCC_data.arbitration_id
                    if SCC_id == 1056:
                        self.set_SCC11(SCC_data)
                    if SCC_id == 1057:
                        SCC_data = self.set_SCC12(SCC_data)
                    if SCC_id == 688:
                        self.set_SAS11(SCC_data)
                    if SCC_id == 909:
                        self.set_FCA11(SCC_data)
                    if SCC_id == 905:
                        SCC_data = self.set_SCC14(SCC_data)
                    self.CCAN.send(SCC_data)
                else:  # Publish Custom BSD Data
                    if time.time() - bsd_time > 0.1:
                        SCC_data = self.set_custom_bsd()
                        self.CCAN.send(SCC_data)
                        bsd_time = time.time()

                # Send CCAN to SCC ( Except CLU11 )
                CCAN_data = self.CCAN.recv(0)
                if CCAN_data != None:
                    if CCAN_data.arbitration_id == 1265:
                        self.set_CLU11(CCAN_data)
                        continue
                    if CCAN_data.arbitration_id == 897:
                        self.set_MDPS11(CCAN_data)
                    if CCAN_data.arbitration_id == 881:
                        self.set_E_EMS11(CCAN_data)
                    if CCAN_data.arbitration_id == 688:
                        self.set_SAS11(CCAN_data)
                    if CCAN_data.arbitration_id == 902:
                        self.set_WHL_SPD11(CCAN_data)
                    if CCAN_data.arbitration_id == 882:
                        self.set_ELECT_GEAR(CCAN_data)

                    if time.time() - cur_time > 0.1:
                        self.can_record_data.data = self.calculate_record()
                        self.can_switch_data.data = self.calculate_switch()
                        self.aeb.data = self.calculate_aeb()
                        self.publisher()
                        cur_time = time.time()

                    self.radar.data = True
                    self.SCC.send(CCAN_data)

            except KeyboardInterrupt:
                exit(0)
            except Exception as e:
                self.radar.data = False
                if time.time() - cur_time > 0.1:
                    self.radar_state.publish(self.radar)
                    cur_time = time.time()
                    print("RADAR ERROR")
                print(e)

    def publisher(self):
        self.can_record.publish(self.can_record_data)
        self.can_switch.publish(self.can_switch_data)
        self.radar_state.publish(self.radar)
        self.aeb_state.publish(self.aeb)

    def calculate_aeb(self):
        aeb = self.aeb.data
        if self.rcv_wheel_speed > 10 and (self.scc11_data['TTC'] <= 2.5 or self.op_fcw):
            self.aeb_time = time.time()
            aeb = True
        elif time.time() - self.aeb_time < 3:
            aeb = True
        else:
            aeb = False
        return aeb

    def calculate_record(self):
        record = self.can_record_data.data
        record[0] = int(self.e_ems11_data['Brake_Pedal_Pos'])
        record[1] = int(self.e_ems11_data['Accel_Pedal_Pos'])
        record[2] = int(self.sas11_data['SAS_Angle'])
        record[3] = self.gear_map[str(
            self.elect_gear_data['Elect_Gear_Shifter'])]
        record[4] = int(self.rpm)
        record[5] = int(self.rcv_wheel_speed)
        return record

    def calculate_switch(self):
        switch = self.can_switch_data.data
        if self.switch_on:
            if self.switch_cnt >= 10:
                switch = [0, 0, 0]
                self.switch_cnt = 0
                self.switch_on = False
            else:
                self.switch_cnt += 1
        if self.e_ems11_data['Brake_Pedal_Pos'] > 20 and self.mode == 1:
            switch[0] = 1
            self.switch_on = True
        elif self.e_ems11_data['Accel_Pedal_Pos'] > 20 and self.mode == 1:
            switch[1] = 1
            self.switch_on = True
        elif abs(self.mdps11_data['CR_Mdps_DrvTq']) > 200 and self.mode == 1:  # 160->220
            switch[2] = 1
            self.switch_on = True
        return switch

    def set_custom_bsd(self):
        data = {
            'custom_bsd_left': self.bsd_array[0], 'custom_bsd_right': self.bsd_array[1]}
        msg = self.db.encode_message('aeye', data)  # Custom AEye data for BSD
        can_msg = can.Message(arbitration_id=1060,
                              data=msg, is_extended_id=False)
        return can_msg

    # Thread 2
    def send_clu(self):
        cnt = 0
        _cnt = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
        data = {'CF_Clu_CruiseSwState': 0.0, 'CF_Clu_CruiseSwMain': 0.0, 'CF_Clu_SldMainSW': 0.0, 'CF_Clu_ParityBit1': 0.0, 'CF_Clu_VanzDecimal': 0.0, 'CF_Clu_Vanz': 0.0,
                'CF_Clu_SPEED_UNIT': 0.0, 'CF_Clu_DetentOut': 1.0, 'CF_Clu_RheostatLevel': 12.0, 'CF_Clu_CluInfo': 0.0, 'CF_Clu_AmpInfo': 0.0, 'CF_Clu_AliveCnt1': 0.0}
        while 1:
            try:
                data['CF_Clu_AliveCnt1'] = _cnt[cnt %
                                                30]  # INT OVERFLOW -> clear 3000
                data['CF_Clu_Vanz'] = self.clu_data['CF_Clu_Vanz']
                data['CF_Clu_RheostatLevel'] = self.clu_data['CF_Clu_RheostatLevel']
                data['CF_Clu_VanzDecimal'] = self.clu_data['CF_Clu_VanzDecimal']
                cnt = cnt + 1
                if cnt >= 3000:
                    cnt = 0
                # Mode Change Test
                data = self.mode_change(data)

                # Vel Change Test
                data = self.vel_change(data, cnt)

                # Spam CLU11 - resume SCC when preceding vehicle start moving
                data = self.resume_scc(data, cnt)

                msg = self.db.encode_message('CLU11', data)
                can_msg = can.Message(
                    arbitration_id=0x4f1, data=msg, is_extended_id=False)
                self.SCC.send(can_msg)

            except KeyboardInterrupt:
                exit(0)
            except:
                print("CLU Exception")
            time.sleep(0.02)

    def set_sw_state(self, current, target):
        # For KIAPI
        # if self.curvature < 300:
        # target = 40

        # For License
        if self.curvature < 400:
            target = min(target, target-int(abs(400-self.curvature)*0.1))

        if current < target:
            return 1
        elif current > target:
            return 2
        elif current == target:
            return 0

    def mode_change(self, data):
        if self.mode != self.prev_mode:
            # Reverse Mode Change Prevent
            if self.scc11_data['MainMode_ACC'] != self.mode:
                data['CF_Clu_CruiseSwMain'] = 1
                self.prev_mode = self.mode
        else:
            data['CF_Clu_CruiseSwMain'] = 0
        return data

    def vel_change(self, data, cnt):
        if self.scc11_data['MainMode_ACC'] == 1 and self.mode == 1 and self.scc12_data['ACCMode'] == False:
            data['CF_Clu_CruiseSwState'] = 2 if data['CF_Clu_CruiseSwState'] == 0 else 0
        elif self.scc11_data['MainMode_ACC'] == 1 and self.mode == 1 and self.scc12_data['ACCMode'] == True:
            data['CF_Clu_CruiseSwState'] = int(self.set_sw_state(
                self.scc11_data['VSetDis'], self.vel)) if cnt % 5 == 0 else 0
        else:
            data['CF_Clu_CruiseSwState'] = 0
        return data

    def resume_scc(self, data, cnt):
        vEgoRaw = self.clu_data['CF_Clu_Vanz']  # Update Ego_velocity
        decimal = self.clu_data['CF_Clu_VanzDecimal']
        if 0. < decimal < 0.5:
            vEgoRaw += decimal

        standstill = vEgoRaw < 0.05

        if standstill:
            if self.last_obj_dist == 0:
                self.last_obj_dist = self.obj_dist

            if abs(self.scc11_data['ACC_ObjDist'] - self.last_obj_dist) > 1:
                data['CF_Clu_CruiseSwState'] = 2 if cnt % 5 == 0 else 0

        # reset lead distnce after the car starts moving
        elif self.last_obj_dist != 0:
            self.last_obj_dist = 0

        return data


if __name__ == '__main__':
    Activate_Signal_Interrupt_Handler()

    Bridge = Bridge()
    th1 = threading.Thread(target=Bridge.bridge)
    th2 = threading.Thread(target=Bridge.send_clu)
    th1.daemon = True
    th2.daemon = True

    try:
        th1.start()
        th2.start()
    except KeyboardInterrupt as e:
        sys.exit(e)

    rospy.spin()
