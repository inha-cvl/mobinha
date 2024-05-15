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
        self.path = []  # 지나온 흔적을 저장할 리스트
        self.last_saved_point = None  # 마지막으로 저장한 위치

        # ROS 노드 초기화 및 subscriber 설정
        rospy.init_node('gps_logger', anonymous=True)
        rospy.Subscriber('/gps/fix', NavSatFix, self.novatel_cb)
        
        # 종료 시그널 처리
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def haversine(self, lat1, lon1, lat2, lon2):
        # 지구의 반지름 (미터)
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c  # 두 점 사이의 거리 (미터)

    def novatel_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        if self.last_saved_point is None:
            # 첫 위치 저장
            self.path.append((self.latitude, self.longitude))
            self.last_saved_point = (self.latitude, self.longitude)
        else:
            # 마지막 저장 위치와의 거리 계산
            last_lat, last_lon = self.last_saved_point
            distance = self.haversine(last_lat, last_lon, self.latitude, self.longitude)
            if distance >= 0.5:  # 50cm 이상인 경우에만 저장
                self.path.append((self.latitude, self.longitude))
                self.last_saved_point = (self.latitude, self.longitude)
        
        print("path length:", len(self.path))

    def save_path_to_file(self, filename):
        # 리스트를 파일로 저장
        with open(filename, 'w') as f:
            for lat, lon in self.path:
                f.write(f"{lat}, {lon}\n")

    def signal_handler(self, signum, frame):
        # 종료 시 호출될 핸들러
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
    rospy.spin()  # ROS 노드를 계속 실행
    
    #
