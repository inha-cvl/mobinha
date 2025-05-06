import can
import cantools
import threading
import time
import rospy
import pymap3d
import signal
import sys
from std_msgs.msg import Float32, Int32, Bool, String

class IONIQ:
    def __init__(self):
        rospy.init_node("controlTest_junmyeong")
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/inha/Documents/catkin_ws/src/mobinha/selfdrive/car/dbc/ioniq/can.dbc')

        # Local variables
        self.PA_enable = 0
        self.LON_enable = 0

        # Control Command
        self.TURN_SIG_RIGHT = 0
        self.TURN_SIG_LEFT = 0
        self.steer = 0
        self.reset = 0
        self.brake = 0
        self.accel = 0
        self.alv_cnt = 0

        # Gateway1 Report
        self.Gway_Cluster_Odometer = None

        # Gateway2 Report
        self.Gway_Lateral_Accel_Speed = None
        self.Gway_Longitudinal_Accel_Speed = None
        self.Gway_Brake_Cylinder_Pressure = None
        self.Gway_Yaw_Rate_Sensor = None

        # Gateway3 Report
        self.velocity_FR = None
        self.velocity_RL = None
        self.velocity_RR = None
        self.velocity_FL = None

        # Gateway3-1 Report
        self.WHL_PlsFLVal = None
        self.WHL_PlsFRVal = None
        self.WHL_PlsRLVal = None
        self.WHL_PlsRRVal = None
        
        # Gateway4 Report
        self.Gway_Steering_Angle = 0
        
        # Gateway5 Report
        self.Gway_Brake_Active = None

        # Gateway6 Report
        self.Gway_GearSelDisp = None
        self.Gway_Accel_Pedal_Position = None

        # Gateway Status Report
        self.PA_Enable_Status = None
        self.LON_Enable_Status = None
        self.Gway_Alive_CNT = None
        
        # Safety Control Report
        self.acc_override = None
        self.brk_override = None
        self.steering_overide = None
        self.safety_status = None
        self.alv_cnt_err = None


        ## SUBSCRIBER
        # GNSS+RTK: NovAtel
        # rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)
        # self.x, self.y, self.z = 0, 0, 0
        # self.roll = 0
        # self.pitch = 0
        # self.yaw = 0
        

        ## PUBLISHER
        # Gateway1 Report
        self.pub_cluster_odometer = rospy.Publisher('/vehicle/cluster_odometer', Float32, queue_size=10)

        # Gateway2 Report
        self.pub_lateral_accel_speed = rospy.Publisher('/vehicle/lateral_accel_speed', Float32, queue_size=10)
        self.pub_longitudinal_accel_speed = rospy.Publisher('/vehicle/longitudinal_accel_speed', Float32, queue_size=10)
        self.pub_brake_pressure = rospy.Publisher('/vehicle/brake_pressure', Float32, queue_size=10)
        self.pub_yaw_rate_sensor = rospy.Publisher('/vehicle/yaw_rate_sensor', Float32, queue_size=10)

        # Gateway3 Report
        self.pub_velocity_FR = rospy.Publisher('/vehicle/velocity_FR', Float32, queue_size=10)
        self.pub_velocity_RL = rospy.Publisher('/vehicle/velocity_RL', Float32, queue_size=10)
        self.pub_velocity_RR = rospy.Publisher('/vehicle/velocity_RR', Float32, queue_size=10)
        self.pub_velocity_FL = rospy.Publisher('/vehicle/velocity_FL', Float32, queue_size=10)

        # Gateway3-1 Report
        self.pub_WHL_PlsFLVal = rospy.Publisher('/vehicle/WHL_PlsFLVal', Float32, queue_size=10) 
        self.pub_WHL_PlsFRVal = rospy.Publisher('/vehicle/WHL_PlsFRVal', Float32, queue_size=10)
        self.pub_WHL_PlsRLVal = rospy.Publisher('/vehicle/WHL_PlsRLVal', Float32, queue_size=10)
        self.pub_WHL_PlsRRVal = rospy.Publisher('/vehicle/WHL_PlsRRVal', Float32, queue_size=10)

        # Gateway4 Report
        self.pub_steering_angle = rospy.Publisher('/vehicle/steering_angle', Float32, queue_size=10)

        # Gateway5 Report
        self.pub_brake_active = rospy.Publisher('/vehicle/brake_active', Bool, queue_size=10)

        # Gateway6 Report
        self.pub_gear_sel_disp = rospy.Publisher('/vehicle/gear_sel_disp', String, queue_size=10)
        self.pub_accel_pedal = rospy.Publisher('/vehicle/accel_pedal_position', Float32, queue_size=10)

        # Gateway Status Report
        self.pub_pa_enable_status = rospy.Publisher('/vehicle/pa_enable_status', Bool, queue_size=10)
        self.pub_lon_enable_status = rospy.Publisher('/vehicle/lon_enable_status', Bool, queue_size=10)
        self.pub_gway_alive_cnt = rospy.Publisher('/vehicle/gway_alive_cnt', Int32, queue_size=10)
        
        # Safety Control Report
        self.pub_acc_override = rospy.Publisher('/vehicle/acc_override', Bool, queue_size=10)
        self.pub_brk_override = rospy.Publisher('/vehicle/brk_override', Bool, queue_size=10)
        self.pub_str_override = rospy.Publisher('/vehicle/steering_override', Bool, queue_size=10)
        self.pub_safety_status = rospy.Publisher('/vehicle/safety_status', Int32, queue_size=10)
        self.pub_alv_cnt_err = rospy.Publisher('/vehicle/alv_cnt_err', Bool, queue_size=10)

    # def novatel_cb(self, msg):
    #     self.x, self.y, self.z = pymap3d.geodetic2enu(
    #         msg.latitude, msg.longitude, 0, self.base_lat, self.base_lon, 0)
    #     self.roll = msg.roll
    #     self.pitch = msg.pitch
    #     self.yaw = 90 - msg.azimuth + 360 if (-270 <= 90 - msg.azimuth <= -180) else 90 - msg.azimuth

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

    def alive_counter(self, alv_cnt):
        return (alv_cnt + 1) % 256


    def reset_trigger(self):
        self.reset = 1

    def longitudinal_cmd(self):
        self.alv_cnt = self.alive_counter(self.alv_cnt)
        signals = {'PA_Enable': self.PA_enable, 'PA_StrAngCmd': self.steer,
                   'LON_Enable': self.LON_enable, 'Target_Brake': self.brake, 'Target_Accel': 20, 
                   'Alive_cnt': self.alv_cnt, 'Reset_Flag': self.reset,
                   'TURN_SIG_LEFT': 0, 'TURN_SIG_RIGHT': 0
                   }
        msg = self.db.encode_message('Control', signals)
        print(signals)
        self.sender(0x210, msg)


    def longitudinal_rcv(self):
        data = self.bus.recv()
        

        # Gateway1 Report
        if data.arbitration_id == 0x520:
            res = self.db.decode_message(1312, data.data)
            self.Gway_Cluster_Odometer = res['Gway_Cluster_Odometer']
            
            # Publish
            self.pub_cluster_odometer.publish(self.Gway_Cluster_Odometer)

        # Gateway2 Report
        if data.arbitration_id == 0x130:
            res = self.db.decode_message(304, data.data)
            self.Gway_Lateral_Accel_Speed = res['Gway_Lateral_Accel_Speed']
            self.Gway_Longitudinal_Accel_Speed = res['Gway_Longitudinal_Accel_Speed']
            self.Gway_Brake_Cylinder_Pressure = res['Gway_Brake_Cylinder_Pressure']
            self.Gway_Yaw_Rate_Sensor = res['Gway_Yaw_Rate_Sensor']

            # Publish
            self.pub_brake_pressure.publish(self.Gway_Brake_Cylinder_Pressure)
            self.pub_lateral_accel_speed.publish(self.Gway_Lateral_Accel_Speed)
            self.pub_longitudinal_accel_speed.publish(self.Gway_Longitudinal_Accel_Speed)
            self.pub_yaw_rate_sensor.publish(self.Gway_Yaw_Rate_Sensor)
            
        # Gateway3 Report
        if data.arbitration_id == 0x280:
            res = self.db.decode_message(0x280, data.data)
            self.velocity_FR = res['Gway_Wheel_Velocity_FR']
            self.velocity_RL = res['Gway_Wheel_Velocity_RL']
            self.velocity_RR = res['Gway_Wheel_Velocity_RR']
            self.velocity_FL = res['Gway_Wheel_Velocity_FL']
            
            # Publish
            self.pub_velocity_FR.publish(self.velocity_FR)
            self.pub_velocity_RL.publish(self.velocity_RL)
            self.pub_velocity_RR.publish(self.velocity_RR)
            self.pub_velocity_FL.publish(self.velocity_FL)
        
        # Gateway3-1 Report
        if data.arbitration_id == 0x281:
            res = self.db.decode_message(0x281, data.data)
            self.WHL_PlsFLVal = res['WHL_PlsFLVal']
            self.WHL_PlsFRVal = res['WHL_PlsFRVal']
            self.WHL_PlsRLVal = res['WHL_PlsRLVal']
            self.WHL_PlsRRVal = res['WHL_PlsRRVal']
            
            # Publish
            self.pub_WHL_PlsFLVal.publish(self.WHL_PlsFLVal)
            self.pub_WHL_PlsFRVal.publish(self.WHL_PlsFLVal)
            self.pub_WHL_PlsRLVal.publish(self.WHL_PlsFLVal)
            self.pub_WHL_PlsRRVal.publish(self.WHL_PlsFLVal)
            
        # Gateway4 Report
        if data.arbitration_id == 0x290:
            res = self.db.decode_message(656, data.data)
            self.Gway_Steering_Angle = res['Gway_Steering_Angle']
            
            # Publish
            self.pub_steering_angle.publish(self.Gway_Steering_Angle)

        # Gateway5 Report
        if data.arbitration_id == 0x260:
            res = self.db.decode_message(608, data.data)
            self.Gway_Brake_Active = res['Gway_Brake_Active']
            
            # Publish
            self.pub_brake_active.publish(self.Gway_Brake_Active)

        # Gateway6 Report
        if data.arbitration_id == 0x170:
            res = self.db.decode_message(368, data.data)
            self.Gway_Accel_Pedal_Position = res['Gway_Accel_Pedal_Position']
            self.Gway_GearSelDisp = res['Gway_GearSelDisp']
            
            # Publish
            self.pub_accel_pedal.publish(self.Gway_Accel_Pedal_Position)
            self.pub_gear_sel_disp.publish(str(self.Gway_GearSelDisp))
            
        # Gateway Status Report
        if data.arbitration_id == 0x211:
            res = self.db.decode_message(data.arbitration_id, data.data)
            self.PA_Enable_Status = res['PA_Enable_Status']
            self.LON_Enable_Status = res['LON_Enable_Status']
            self.Gway_Alive_CNT = res['GWAY_Alive_CNT']

            # Publish PA and LON status
            self.pub_pa_enable_status.publish(self.PA_Enable_Status)
            self.pub_lon_enable_status.publish(self.LON_Enable_Status)    
            self.pub_gway_alive_cnt.publish(self.Gway_Alive_CNT)

        # Safety Control Report
        if data.arbitration_id == 0x310:
            res = self.db.decode_message(784, data.data)
            self.acc_override = res['Accel_Override']
            self.brk_override = res['Break_Override']
            self.str_override = res['Steering_Overide']
            self.safety_status = res['Safety_Status']
            self.alv_cnt_err = res['Alive_Count_ERR']
            
            # Publish
            self.pub_acc_override.publish(self.acc_override)
            self.pub_brk_override.publish(self.brk_override)
            self.pub_str_override.publish(self.steering_overide)
            self.pub_safety_status.publish(self.safety_status)
            self.pub_alv_cnt_err.publish(self.alv_cnt_err)
        

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

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Exiting gracefully...')
    rospy.signal_shutdown('Exiting')
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    IONIQ = IONIQ()
    t1 = threading.Thread(target=IONIQ.daemon)
    t2 = threading.Thread(target=IONIQ.state_controller)

    t1.start()
    t2.start()

    t1.join()
    t2.join()