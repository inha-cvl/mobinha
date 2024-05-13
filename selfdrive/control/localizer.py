#!/usr/bin/python
from selfdrive.visualize.rviz_utils import *
import tf
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D


class Localizer:
    def __init__(self):
        self.ego_car = EgoCarViz()
        self.br = tf.TransformBroadcaster()

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
        self.br.sendTransform((-0.5, 0, 1.2),(0, 0, 0, 1), rospy.Time.now(), 'Pandar64', 'ego_car')

        self.pub_ego_car.publish(self.ego_car)
        self.pub_enu_pose.publish(CS.position.x, CS.position.y, CS.yawRate)