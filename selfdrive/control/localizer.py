#!/usr/bin/python
from selfdrive.visualize.rviz_utils import *
import tf
import tf2_ros
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D, TransformStamped

class Localizer:
    def __init__(self):
        self.ego_car = EgoCarViz()
        self.br = tf.TransformBroadcaster()

        # calibration
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        static_transforms = [
            ((1.5275, 0.0, 0.0), (0, 0, 0, 1), 'ego_car', 'gps'),   # center
            ((1.06, 0, 1.22), rotate_quaternion_yaw((0, 0, 0, 1), -2.1), 'Pandar64', 'gps'),   # debug
            #((1.06, 0, 1.22), rotate_quaternion_yaw((0, 0, 0, 1), -2.1), 'Pandar64', 'gps') # drive
            #((1.8, 0.0, 1.0), (0, 0, 0, 1), 'front', 'gps'),        # front camera
            #((1.7, 0.7, 1.0), (0, 0, 0, 1), 'left_front', 'gps'),   # left 
            #((1.4, -0.7, 1.0), (0, 0, 0, 1), 'right_rear', 'gps')   # right
        ]
        self.publish_static_tfs(static_transforms)

        self.pub_ego_car = rospy.Publisher('/mobinha/control/ego_car', Marker, queue_size=1)
        self.pub_enu_pose = rospy.Publisher('/veh_pose', Pose2D, queue_size=1)

    # calibration
    def publish_static_tfs(self, transforms):
        static_transformStamped_vec = []
        for translation, rotation, child_frame, parent_frame in transforms:
            static_transformStamped = TransformStamped()
            #static_transformStamped.header.stamp = rospy.Time(0)
            static_transformStamped.header.frame_id = parent_frame
            static_transformStamped.child_frame_id = child_frame
            static_transformStamped.transform.translation.x = translation[0]
            static_transformStamped.transform.translation.y = translation[1]
            static_transformStamped.transform.translation.z = translation[2]
            static_transformStamped.transform.rotation.x = rotation[0]
            static_transformStamped.transform.rotation.y = rotation[1]
            static_transformStamped.transform.rotation.z = rotation[2]
            static_transformStamped.transform.rotation.w = rotation[3]
            static_transformStamped_vec.append(static_transformStamped)
        
        self.static_br.sendTransform(static_transformStamped_vec)
    
    def run(self, sm):
        CS = sm.CS
        quaternion = tf.transformations.quaternion_from_euler(
            math.radians(CS.rollRate), math.radians(CS.pitchRate), math.radians(CS.yawRate))  # RPY
        self.br.sendTransform(
            (CS.position.x, CS.position.y, CS.position.z),
            (quaternion[0], quaternion[1],
                quaternion[2], quaternion[3]),
            CS.timestamp,
            'gps',
            'world'
        )
        
        self.pub_ego_car.publish(self.ego_car)
        self.pub_enu_pose.publish(CS.position.x, CS.position.y, CS.yawRate)