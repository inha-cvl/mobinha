#!/usr/bin/env python

import rospy
import signal
import math
from sensor_msgs.msg import NavSatFix

import matplotlib.pyplot as plt

class GPSLogger:
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.path = [] 
        self.last_saved_point = None 

        rospy.init_node('gps_logger', anonymous=True)
        rospy.Subscriber('/gps/fix', NavSatFix, self.novatel_cb)
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c 

    def novatel_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        if self.last_saved_point is None:
            self.path.append((self.latitude, self.longitude))
            self.last_saved_point = (self.latitude, self.longitude)
        else:
            last_lat, last_lon = self.last_saved_point
            distance = self.haversine(last_lat, last_lon, self.latitude, self.longitude)
            if distance >= 0.1:  # 50cm 이상인 경우에만 저장
                self.path.append((self.latitude, self.longitude))
                self.last_saved_point = (self.latitude, self.longitude)
        
        print("path length:", len(self.path))

    def save_path_to_file(self, filename):
        with open(filename, 'w') as f:
            for lat, lon in self.path:
                f.write(f"{lat}, {lon}\n")


    def signal_handler(self, signum, frame):
        rospy.loginfo("Saving path to file...")
        self.save_path_to_file("path_log.txt")
        rospy.loginfo("Path saved. Exiting...")
        plt.plot([el[0] for el in self.path], [el[1] for el in self.path], "r", label="path")
        plt.legend()
        plt.grid(True)
        plt.show()
        rospy.signal_shutdown("Path saved")

if __name__ == "__main__":
    logger = GPSLogger()
    rospy.spin()
    
