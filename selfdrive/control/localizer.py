#!/usr/bin/python
from selfdrive.visualize.rviz_utils import *
import tf
import tf2_ros
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D


class Localizer:
    def __init__(self):
        self.ego_car = EgoCarViz()
        self.br = tf.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_ego_car = rospy.Publisher('/mobinha/control/ego_car', Marker, queue_size=1)
        self.pub_enu_pose = rospy.Publisher('/veh_pose', Pose2D, queue_size=1)

    def run(self, sm):
        CS = sm.CS
        quaternion = tf.transformations.quaternion_from_euler(
            math.radians(CS.rollRate), math.radians(CS.pitchRate), math.radians(CS.yawRate))  # RPY
        self.br.sendTransform(
            (CS.position.x, CS.position.y, CS.position.z),
            (quaternion[0], quaternion[1],
                quaternion[2], quaternion[3]),
            rospy.Time.now(),
            'ego_car',
            'world'
        )
        # INS부터 각 바퀴까지의 거리 정의
        self.br.sendTransform(
            (1.91, 0.94, 0),
            (0,0,0,1),
            rospy.Time.now(),
            'fl',
            'ego_car'
        )
        self.br.sendTransform(
            (1.91, -0.94, 0),
            (0,0,0,1),
            rospy.Time.now(),
            'fr',
            'ego_car'
        )
        self.br.sendTransform(
            (0, 0.94, 0),
            (0,0,0,1),
            rospy.Time.now(),
            'rl',
            'ego_car'
        )
        self.br.sendTransform(
            (0, -0.94, 0),
            (0,0,0,1),
            rospy.Time.now(),
            'rr',
            'ego_car'
        )
        
        self.br.sendTransform((-0.5, 0, 1.2),(0, 0, 0, 1), rospy.Time.now(), 'Pandar64', 'ego_car')

        self.pub_ego_car.publish(self.ego_car)
        self.pub_enu_pose.publish(CS.position.x, CS.position.y, CS.yawRate)
