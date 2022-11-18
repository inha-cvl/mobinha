#!/usr/bin/python
from selfdrive.visualize.viz import *
import tf
import math
import os

import rospy
from visualization_msgs.msg import Marker

dir_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(dir_path)


class Localizer:
    def __init__(self):
        self.ego_car = EgoCarViz()
        self.br = tf.TransformBroadcaster()
        self.pub_ego_car = rospy.Publisher('/ego_car', Marker, queue_size=1)

    def run(self, sm):
        CS = sm.CS
        quaternion = tf.transformations.quaternion_from_euler(
            0.0, 0.0, math.radians(CS.yawRate))
        self.br.sendTransform(
            (CS.position.x, CS.position.y, 0.0),
            (quaternion[0], quaternion[1],
                quaternion[2], quaternion[3]),
            rospy.Time.now(),
            'ego_car',
            'world'
        )
        self.pub_ego_car.publish(self.ego_car)
