#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import signal
import sys

# 데이터를 저장할 리스트 초기화
x_data, y_data, z_data, w_data = [], [], [], []

# SIGINT 핸들러
def signal_handler(sig, frame):
    # 종료 시 데이터를 파일로 저장
    print("SIGINT received, saving data to file...")

    # 데이터를 텍스트 파일로 저장
    with open("pose_data.txt", "w") as f:
        f.write("margined_safety_distance (x), current_s (y), target_v (z), CS.vEgo (w)\n")
        for i in range(len(x_data)):
            f.write(f"{x_data[i]}, {y_data[i]}, {z_data[i]}, {w_data[i]}\n")

    print("Data saved to pose_data.txt")
    sys.exit(0)

# ROS 메시지 콜백 함수
def pose_callback(msg):
    global x_data, y_data, z_data, w_data

    # 메시지에서 데이터를 수신하여 리스트에 추가
    x_data.append(msg.orientation.x)
    y_data.append(msg.orientation.y)
    z_data.append(msg.orientation.z)
    w_data.append(msg.orientation.w)

    # 수신된 데이터 출력 (옵션)
    # rospy.loginfo(f"Received: sx={msg.orientation.x}, y={msg.orientation.y}, z={msg.orientation.z}, w={msg.orientation.w}")

def listener():
    # ROS 노드 초기화
    rospy.init_node('pose_listener', anonymous=True)

    # Pose 메시지 토픽 구독
    rospy.Subscriber('/mobinha/acc_plot', Pose, pose_callback)

    # 종료될 때까지 대기
    rospy.spin()

if __name__ == '__main__':
    # SIGINT 핸들러 설정
    signal.signal(signal.SIGINT, signal_handler)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
