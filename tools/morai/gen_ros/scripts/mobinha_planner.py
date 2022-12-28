#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import signal
import time

from std_msgs.msg import String
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, CtrlCmd
import tf
from math import pi, radians

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseArray


def signal_handler(sig, frame):
    sys.exit(0)


class MoraiPlanner():
    def __init__(self):
        self.state = 'WAITING'
        #publisher
        self.ctrl_pub = rospy.Publisher(
            '/ctrl_cmd', CtrlCmd, queue_size=1)  # Vehicl Control
        self.obj_list_pub = rospy.Publisher(
            '/morai/object_list', PoseArray, queue_size=1)
        self.ego_topic_pub = rospy.Publisher(
            '/morai/ego_topic', Pose, queue_size=1)
        #subscriber
        sub_state = rospy.Subscriber('/state', String, self.state_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus,
                         self.statusCB)  # Vehicl Status Subscriber
        rospy.Subscriber('/wheel_angle', Float32, self.wheel_angle_cb)
        rospy.Subscriber('/accel_brake', Float32, self.accel_brake_cb)
        rospy.Subscriber("/Object_topic", ObjectStatusList,
                         self.object_topic_cb)

    def planning(self):
        self.wheel_angle = 0
        self.accel_brake = 0
        self.is_status = False  # 차량 상태 점검

        ctrl_msg = CtrlCmd()
        ctrl_msg = self.init_ctrl_cmd(ctrl_msg)

        while True:
            if self.state == 'START':
                ctrl_msg.steering = self.wheel_angle
                control_input = self.accel_brake

                if control_input > 0:
                    ctrl_msg.accel = control_input
                    ctrl_msg.brake = 0
                else:
                    ctrl_msg.accel = 0
                    ctrl_msg.brake = -control_input

                self.ctrl_pub.publish(ctrl_msg)

            elif self.state == 'INITIALIZE':
                ctrl_msg = self.init_ctrl_cmd(ctrl_msg)
                self.ctrl_pub.publish(self.init_ctrl_cmd(ctrl_msg))

            elif self.state == 'FINISH':
                ctrl_msg = self.init_ctrl_cmd(ctrl_msg)
                self.ctrl_pub.publish(self.init_ctrl_cmd(ctrl_msg))
                return 1
            else:
                self.ctrl_pub.publish(ctrl_msg)
            time.sleep(0.03)

    def init_ctrl_cmd(self, ctrl_cmd):
        ctrl_cmd.steering = 0
        ctrl_cmd.accel = 0
        ctrl_cmd.brake = 1.0
        return ctrl_cmd

    def wheel_angle_cb(self, msg):
        if msg.data != 0:
            self.wheel_angle = radians(msg.data)

    def accel_brake_cb(self, msg):
        if msg.data != 0:
            self.accel_brake = msg.data

    def statusCB(self, data):  # Vehicle Status Subscriber
        self.status_msg = data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                         tf.transformations.quaternion_from_euler(
                             0, 0, (self.status_msg.heading)/180*pi),
                         rospy.Time.now(),
                         "gps",
                         "map")
        pose = Pose()
        pose.position.x = data.position.x
        pose.position.y = data.position.y
        pose.position.z = data.position.z
        self.ego_topic_pub.publish(pose)
        self.velocity = data.velocity.x
        self.is_status = True

    def object_topic_cb(self, data):
        object_list = PoseArray()
        for obj in data.npc_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            object_list.poses.append(pose)
        for obj in data.obstacle_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            object_list.poses.append(pose)
        for obj in data.pedestrian_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            object_list.poses.append(pose)
        self.obj_list_pub.publish(object_list)

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Morai_Planner', anonymous=True)

    m = MoraiPlanner()
    print("[{}] Created".format(m.__class__.__name__))

    try:
        if m.planning() == 1:
            print("[{}] Over".format(m.__class__.__name__))
            time.sleep(3)
            sys.exit(0)
    except Exception as e:
        print("[{} Error]".format(m.__class__.__name__), e)
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(m.__class__.__name__))
        sys.exit(0)
