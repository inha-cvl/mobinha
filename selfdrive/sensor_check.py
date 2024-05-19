import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import Float32MultiArray, Int8, Int16MultiArray
import numpy as np
from novatel_oem7_msgs.msg import BESTGNSSPOS, INSPVA
from geometry_msgs.msg import PoseArray
from jsk_recognition_msgs.msg import BoundingBoxArray

# Camera & Lidar
class SensorCheck:
    def __init__(self, topic_name, msg_type, hz_thresh):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.message = ''
        self.message_count = 0
        self.state = ''
        self.hz = 0.0
        self.hz_thresh = hz_thresh

        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self, msg):
        self.current_time = rospy.Time.now()
        self.message = msg.data
        self.message_count += 1

    def get_hz(self):
        if self.current_time != self.last_time and len(self.message) != 0:
            self.hz = self.message_count / (self.current_time - self.last_time).to_sec()
            self.message_count = 0
            self.last_time = self.current_time
        else:
            self.hz = 0.0
        return self.hz
    
    def check(self):
        if self.get_hz() > self.hz_thresh:
            return 1
        else:
            return 0
        
class GPSCheck:
    def __init__(self, topic_name, msg_type, hz_thresh, lat_thresh, lon_thresh):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.current_seq = 0
        self.last_seq = 0
        self.lat_std = 0.0
        self.lon_std = 0.0
        self.hgt_std = 0.0
        self.message_count = 0
        self.sol_age = 0.0
        self.hz = 0.0
        self.hz_thresh = hz_thresh
        self.lat_thresh = lat_thresh
        self.lon_thresh = lon_thresh

        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self, msg):
        self.current_time = rospy.Time.now()
        self.current_seq = msg.header.seq
        self.sol_age = msg.sol_age
        self.lat_std = msg.lat_stdev
        self.lon_std = msg.lon_stdev
        self.hgt_std = msg.hgt_stdev

        self.message_count += 1
        
    def get_hz(self):
        if self.current_time != self.last_time and self.current_seq != self.last_seq and self.sol_age < 1.0:
            self.hz = self.message_count / (self.current_time - self.last_time).to_sec()
            self.message_count = 0
            self.last_time = self.current_time
            self.last_seq = self.current_seq
        else:
            self.hz = 0.0
        return self.hz

    def check(self):
        if self.get_hz() > self.hz_thresh and self.lat_std < self.lat_thresh and self.lon_std < self.lon_thresh:
            return 1
        else:
            return 0
    
class INSCheck:
    def __init__(self, topic_name, msg_type, hz_thresh):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.current_seq = 0
        self.last_seq = 0
        self.message_count = 0
        self.lat = 0.0
        self.pre_lat = 0.0
        self.hz = 0.0
        self.hz_thresh = hz_thresh

        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self, msg):
        self.current_time = rospy.Time.now()
        self.current_seq = msg.header.seq
        self.lat = msg.latitude
        self.message_count += 1
        
    def get_hz(self):
        if self.current_time != self.last_time and self.current_seq != self.last_seq and self.lat != 0.0 and self.pre_lat != self.lat:
            self.hz = self.message_count / (self.current_time - self.last_time).to_sec()
            self.message_count = 0
            self.last_time = self.current_time
            self.last_seq = self.current_seq
            self.pre_lat = self.lat
        else:
            self.hz = 0.0
        return self.hz

    def check(self):
        if self.get_hz() > self.hz_thresh:
            return 1
        else:
            return 0

class CanCheck:
    def __init__(self, topic_name, msg_type):
        self.current_time = rospy.Time.now()
        self.state = 1
        
        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self, msg):
        self.current_time = rospy.Time.now()
        self.state = msg.data
    
    def check(self):
        if (rospy.Time.now() - self.current_time).to_sec() < 1:
            return self.state
        else:
            return 0

class PlanningCheck:
    def __init__(self, topic_name, msg_type):
        self.current_time = rospy.Time.now()
        self.pp = 0
        self.lgp = 0

        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self, msg):
        self.current_time = rospy.Time.now()
        self.pp = msg.data[0]
        self.lgp = msg.data[1]
    
    def check(self):
        if (rospy.Time.now() - self.current_time).to_sec() < 2.0 and self.pp == 1 and self.lgp == 1:
            return 1
        else:
            return 0
            
class ObjectCheck:
    def __init__(self, topic_name, msg_type, hz_thresh):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.message = ''
        self.message_count = 0
        self.state = ''
        self.hz = 0.0
        self.hz_thresh = hz_thresh

        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self, msg):
        self.current_time = rospy.Time.now()
        self.message = msg.poses
        self.message_count += 1

    def get_hz(self):
        if self.current_time != self.last_time:
            self.hz = self.message_count / (self.current_time - self.last_time).to_sec()
            self.message_count = 0
            self.last_time = self.current_time
        else:
            self.hz = 0.0
        return self.hz
    
    def check(self):
        if self.get_hz() > self.hz_thresh:
            return 1
        else:
            return 0

class ClusterCheck:
    def __init__(self, topic_name, msg_type, hz_thresh):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.message = ''
        self.message_count = 0
        self.state = ''
        self.hz = 0.0
        self.hz_thresh = hz_thresh

        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self, msg):
        self.current_time = rospy.Time.now()
        self.message = msg.boxes
        self.message_count += 1

    def get_hz(self):
        if self.current_time != self.last_time: #and len(self.message) != 0:
            self.hz = self.message_count / (self.current_time - self.last_time).to_sec()
            self.message_count = 0
            self.last_time = self.current_time
        else:
            self.hz = 0.0
        return self.hz
    
    def check(self):
        if self.get_hz() > self.hz_thresh:
            return 1
        else:
            return 0
        
def main():
    rospy.init_node('sensor_diagnostics')
    pub = rospy.Publisher('sensor_check', Int16MultiArray, queue_size=10)
    
    cam = SensorCheck('/gmsl_camera/dev/video0/compressed', CompressedImage, 20)
    lidar = SensorCheck('/cloud_segmentation/nonground', PointCloud2, 5)
    gps = GPSCheck('/novatel/oem7/bestgnsspos', BESTGNSSPOS, 7, 1.0, 1.0)
    ins = INSCheck('/novatel/oem7/inspva', INSPVA, 20)
    can = CanCheck('/mobinha/car/gateway_state',Int8)
    planning = PlanningCheck('/mobinha/planning_state', Int16MultiArray)
    tl = ObjectCheck('/mobinha/perception/camera/bounding_box', PoseArray, 1)
    cluster = ClusterCheck('/mobinha/perception/lidar/track_box', BoundingBoxArray, 3)
    
    sensor_check = Int16MultiArray()

    while not rospy.is_shutdown():
        # cam1, cam2, cam3, lidar, gps, ins, can, perception, planning
        # 1 : problem / 0 : no problem
        sensor_check.data = [cam.check(), 1, 1, lidar.check(), gps.check(), ins.check(), can.check(), 
                             int(tl.check() == cluster.check() == 1), planning.check()]
        #sensor_check.data = [cam.check(), 1, 1, 1, 1, 1, 1, 1, 1]
        pub.publish(sensor_check)
        rospy.sleep(0.2) # 5hz
        
if __name__ == '__main__':
    main()
