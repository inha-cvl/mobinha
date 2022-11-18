import rospy
import os
import signal
import sys

from sensor_msgs.msg import CompressedImage, PointCloud2
from visualization_msgs.msg import MarkerArray
from sbg_driver.msg import SbgEkfNav, SbgEkfEuler, SbgGpsPos
from std_msgs.msg import Int8MultiArray, Float32MultiArray, Bool, Int8
from geometry_msgs.msg import Point

class Final:
    def __init__(self):

        rospy.init_node('sensor_check', anonymous=True)
        rospy.Subscriber('/gmsl_camera/dev/video0/compressed', CompressedImage, self.video0_Callback)
        rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.LiDAR_points_Callback)
        rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.sbg_ekf_Callback)
        rospy.Subscriber('/sbg/gps_pos', SbgGpsPos, self.GPS_pos_Callback)
        rospy.Subscriber('/sbg/ekf_nav', SbgEkfNav, self.EKF_nav_Callback)
        rospy.Subscriber('/Camera/Front60/od_bbox', Float32MultiArray, self.Video0_result_Callback)
        rospy.Subscriber('/pillar_marker_array', MarkerArray, self.LiDAR_result_Callback)
        rospy.Subscriber('/unstable_lane',Bool , self.lane_state_callback)
        rospy.Subscriber('/radar',Bool , self.radar_callback)
        rospy.Subscriber('/lkas',Bool , self.lkas_callback)        
        rospy.Subscriber('/mode', Int8, self.mode_callback)

        self.gps_check = rospy.Publisher('/gps_accuracy', Point,  queue_size=1)
        self.sensor_check = rospy.Publisher('/sensor_state', Int8MultiArray, queue_size=1)
        self.system_check = rospy.Publisher('/system_state', Int8MultiArray, queue_size=1)

        self.Video0_count = 0
        self.LiDAR_points_count = 0
        self.Sbg_euler_count = 0
        self.Video0_result_count = 0
        self.LiDAR_result_count = 0
        self.GPS_error_type = 0
        self.FIX_error_type = 0
        self.GPS_accuracy = 0

        self.unstable_lane = 0
        self.radar = 0 # false
        self.lkas = False
        self.mode = 0
        self.test_lane = 0

    def mode_callback(self, msg):
        self.mode = msg.data
    def video0_Callback(self, msg):
        self.Video0_count += 1
    def LiDAR_points_Callback(self, msg):
        self.LiDAR_points_count += 1
    def sbg_ekf_Callback(self, msg):
        self.Sbg_euler_count += 1
    def GPS_pos_Callback(self, msg):
        self.FIX_error_type = 1 if msg.diff_age > 500 else 0
    def EKF_nav_Callback(self, msg):
        self.GPS_error_type = 1 if msg.position_accuracy.x > 0.7 else 0
        self.GPS_accuracy = round(float(msg.position_accuracy.x), 3)
    def lane_state_callback(self, msg):
        self.unstable_lane = int(msg.data)
    def radar_callback(self, msg):
        self.radar = int(msg.data)
    def lkas_callback(self, msg):
        self.lkas = msg.data
    def Video0_result_Callback(self, msg):
        self.Video0_result_count += 1
    def LiDAR_result_Callback(self, msg):
        self.LiDAR_result_count += 1

    def checker(self):
        sensor_state = Int8MultiArray()
        system_state = Int8MultiArray()
        gps_accuracy = Point()
        print("="*50)

        if self.Video0_count > 23:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])
        print("Camera : {}Hz".format(self.Video0_count))

        if self.lkas: # lkas False == openpilot Fault
            sensor_state.data.extend([True])
        elif not self.lkas:
            sensor_state.data.extend([False])
        print("LKAS : {}".format(self.lkas))

        if self.LiDAR_points_count > 6:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])
        print("LiDAR : {}Hz".format(self.LiDAR_points_count))

        if self.GPS_error_type == 0 and self.FIX_error_type == 0:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])
        print("GPS : {}, {}".format(self.GPS_error_type, self.FIX_error_type))
        gps_accuracy.x = not self.GPS_error_type
        gps_accuracy.y = self.GPS_accuracy

        if self.Sbg_euler_count > 18:
            sensor_state.data.extend([False])
        elif self.Sbg_euler_count <= 18:
            sensor_state.data.extend([True])
        print("INS : {}Hz".format(self.Sbg_euler_count))

        if self.radar == 1: # RADAR true
            sensor_state.data.extend([False])
        elif self.radar == 0: # RADAR false
            sensor_state.data.extend([True])
        print("Radar : {}".format(self.radar))


        #System Error Check
        if self.Video0_result_count > 20:
            system_state.data.extend([False])
        else:
            system_state.data.extend([True])
        print("Camera result : {}Hz".format(self.Video0_result_count))

        if self.LiDAR_result_count > 3:
            system_state.data.extend([False])
        else:
            system_state.data.extend([True])
        print("LiDAR result : {}Hz".format(self.LiDAR_result_count))

        if self.unstable_lane and self.mode == 0:
            system_state.data.extend([True])
            self.test_lane = 1
            print("My Lane BAD")
        else:
            system_state.data.extend([False])
            self.test_lane = 0

        self.Video0_count = 0
        self.LiDAR_points_count = 0
        self.Sbg_euler_count = 0
        self.Video0_result_count = 0
        self.LiDAR_result_count = 0
        self.GPS_error_type = 0
        self.FIX_error_type = 0

        self.talker(sensor_state, system_state, gps_accuracy)

    def talker(self, sensorstate, systemstate, gpsaccuracy):
        print("="*50)
        print("Camera, LKAS, LiDAR, GPS, INS, RADARv")
        print(sensorstate.data)
        print("Cam_result, LiDAR_result, Unstable Lane")
        print(systemstate.data)

        # ---
        # This is for Test  ( Have to Remove !!!!!! )
        #sensorstate.data = [0,0,0,0,0,0]
        #systemstate.data = [0, 0, self.test_lane]
        # ---- 
        
        self.sensor_check.publish(sensorstate)
        self.system_check.publish(systemstate)
        self.gps_check.publish(gpsaccuracy)

        self.sensor_check.data = []
        self.system_check.data = []


def signal_handler(sig, frame):
    print('\nPressed CTRL + C !')
    sys.exit(0)


if __name__ == "__main__":
    mm = Final()
    print(" Check Start !")
    while not rospy.is_shutdown():
        try:
            rospy.sleep(1)
            signal.signal(signal.SIGINT, signal_handler)
            os.system('clear')
            mm.checker()
        except Exception as e:
            print(e)
            print(type(e))
            exit()
