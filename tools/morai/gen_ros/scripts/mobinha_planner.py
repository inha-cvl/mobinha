#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import signal
import time

from std_msgs.msg import String
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, CtrlCmd, GetTrafficLightStatus, Lamps
import tf
from math import pi, radians

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose, PoseArray


# def signal_handler(sig, frame):
#     sys.exit(0)


class MoraiPlanner():
    def __init__(self):
        self.state = 'WAITING'
        self.CM = None
        # publisher
        self.ctrl_pub = rospy.Publisher(
            '/ctrl_cmd', CtrlCmd, queue_size=1)  # Vehicl Control
        self.lamp_pub = rospy.Publisher('/lamps', Lamps, queue_size=1)
        self.obj_list_pub = rospy.Publisher(
            '/morai/object_list', PoseArray, queue_size=1)
        self.traffic_light_pub = rospy.Publisher(
            '/morai/traffic_light', PoseArray, queue_size=1)
        self.ego_topic_pub = rospy.Publisher(
            '/morai/ego_topic', Pose, queue_size=1)
        rospy.Subscriber(
            '/mobinha/visualize/system_state', String, self.state_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus,
                         self.statusCB)
        rospy.Subscriber("/Object_topic", ObjectStatusList,
                         self.object_topic_cb)
        rospy.Subscriber("/GetTrafficLightStatus",
                         GetTrafficLightStatus, self.get_traffic_light_status_cb)
        rospy.Subscriber('/mobinha/planning/blinker', Int8, self.blinker_cb)
        self.ctrl_msg = CtrlCmd()
        self.lamps = Lamps()

    def planning(self, CM):
        self.wheel_angle = 0
        self.accel_brake = 0
        self.is_status = False  # 차량 상태 점검

        self.ctrl_msg = self.init_ctrl_cmd(self.ctrl_msg)
        self.CM = CM
        while True:
            self.CM.update()
            if self.state == 'START':
                self.ctrl_msg = self.set_ctrl_cmd(self.ctrl_msg)
                self.ctrl_pub.publish(self.ctrl_msg)
            elif self.state == 'INITIALIZE':
                self.ctrl_msg = self.init_ctrl_cmd(self.ctrl_msg)
                self.ctrl_pub.publish(self.ctrl_msg)
            elif self.state == 'OVER':
                self.ctrl_msg = self.init_ctrl_cmd(self.ctrl_msg)
                self.ctrl_pub.publish(self.ctrl_msg)
                return 1
            else:
                self.ctrl_pub.publish(self.ctrl_msg)
            time.sleep(0.02)

    def init_ctrl_cmd(self, ctrl_cmd):
        ctrl_cmd.steering = 0
        ctrl_cmd.accel = 0
        ctrl_cmd.brake = 1.0
        return ctrl_cmd

    def blinker_cb(self, msg):
        self.lamps.turnSignal = msg.data
        self.lamp_pub.publish(self.lamps)

    def clip(self, x, lo, hi):
        return max(lo, min(hi, x))

    def rmin(self, x, hi):
        return min(hi, x)

    def set_ctrl_cmd(self, ctrl_cmd):
        ctrl_cmd.steering = radians(self.CM.CC.actuators.steer)
        ctrl_cmd.accel = self.rmin(self.CM.CC.actuators.accel*5, 100)*0.01
        ctrl_cmd.brake = self.rmin(self.CM.CC.actuators.brake*80/65, 100)*0.01
        # if 0 < self.CM.CC.actuators.brake/10*0.11 < 0.01:
        #     ctrl_cmd.brake = 0.003
        # else:
        #     ctrl_cmd.brake = self.rmin(1.1*(self.CM.CC.actuators.brake/10*0.11)-0.011, 0.11)
        return ctrl_cmd

    def statusCB(self, data):  # Vehicle Status Subscriber
        self.status_msg = data
        pose = Pose()
        pose.position.x = data.position.x
        pose.position.y = data.position.y
        pose.position.z = data.position.z
        pose.orientation.w = data.velocity.x
        self.ego_topic_pub.publish(pose)
        self.velocity = data.velocity.x
        self.heading = data.heading
        self.is_status = True

    def object_topic_cb(self, data):
        object_list = PoseArray()
        for obj in data.npc_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        for obj in data.obstacle_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        for obj in data.pedestrian_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        self.obj_list_pub.publish(object_list)

    def get_traffic_light_status_cb(self, data):
        traffic_light_list = PoseArray()

        pose = Pose()
        st = data.trafficLightStatus
        stt = 10
        if st == 1:  # Red
            stt = 10
        elif st == 4:  # Yellow
            stt = 11
        elif st == 16:  # Green
            stt = 9
        elif st == 32:  # Green Left
            stt = 12
        elif st == 33:  # Red wotj Greem :left
            stt = 12
        elif st == 48:  # Green with Green Left
            stt = 14
        elif st == 20:  # Yellow with Green
            stt = 12
        elif st == 36:  # Yellow with Green Left
            stt = 12
        elif st == 5:  # Red with YUellow
            stt = 13

        #cls, size, prob
        pose.position.x = stt
        pose.position.y = 0.3
        pose.position.z = 0.8
        traffic_light_list.poses.append(pose)
        self.traffic_light_pub.publish(traffic_light_list)

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALIZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)


def run(CM):
    #signal.signal(signal.SIGINT, signal_handler)
    m = MoraiPlanner()
    print("[{}] Created".format(m.__class__.__name__))

    try:
        if m.planning(CM) == 1:
            print("[{}] Over".format(m.__class__.__name__))
            time.sleep(4)
            sys.exit(0)
    except Exception as e:
        print("[{} Error]".format(m.__class__.__name__), e)
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(m.__class__.__name__))
        sys.exit(0)
