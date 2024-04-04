#!/usr/bin/python
from selfdrive.visualize.rviz_utils import *
import tf
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D


# mingu
def square_pyramid_marker(frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id  # 원하는 프레임 설정
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 1.0  # 사각뿔의 크기 설정
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 0.7  # 투명도 설정
    marker.color.r = 0.0  # 색상 설정
    marker.color.g = 0.0
    marker.color.b = 1.0

    # 사각뿔의 꼭지점 좌표 설정
    points = [Point(0, 0, 0),  # 꼭대기
                Point(35, 30, 5),  # 기준점 1
                Point(35, -30, 5),  # 기준점 2
                Point(35, 30, -3),  # 기준점 3
                Point(35, -30, -3)]  # 기준점 4

    # 사각뿔의 면 설정
    marker.points = [points[0], points[1], points[2],  # 삼각형 1
                        points[0], points[2], points[3],  # 삼각형 2
                        points[0], points[3], points[4],  # 삼각형 3
                        points[0], points[4], points[1]]  # 삼각형 4
    
    return marker


class Localizer:
    def __init__(self):
        self.ego_car = EgoCarViz()
        self.br = tf.TransformBroadcaster()

        self.pub_ego_car = rospy.Publisher('/mobinha/control/ego_car', Marker, queue_size=1)
        self.pub_enu_pose = rospy.Publisher('/veh_pose', Pose2D, queue_size=1)

        # mingu
        self.pub_cam1 = rospy.Publisher('cam1_marker', Marker, queue_size=1)
        self.pub_cam2 = rospy.Publisher('cam2_marker', Marker, queue_size=1)
        self.pub_cam3 = rospy.Publisher('cam3_marker', Marker, queue_size=1)

    def run(self, sm):
        CS = sm.CS

        quaternion = tf.transformations.quaternion_from_euler(
            math.radians(CS.rollRate), math.radians(CS.pitchRate), math.radians(CS.yawRate))  # RPY


        # mingu
        self.br.sendTransform((1.054, 0, 1.221), (0, 0, 0, 1), rospy.Time.now(), 'Pandar64', 'ego_car')

        # mingu
        # enu 좌표계로 변환 된 position 이기에 ego 차량 frame 기준 위치를 옮기고 싶으면 NAS에서 calibration 해줘야 함
        # 나중에는 base_link 또는 ego_car frame을 차의 원점 좌표로 만들어서 캘리브레이션 해야 함
        self.br.sendTransform(
            (CS.position.x, CS.position.y, CS.position.z),
            (quaternion[0], quaternion[1],
                quaternion[2], quaternion[3]),
            rospy.Time.now(),
            'ego_car',
            'world'
        )
        
        #self.br.sendTransform((2.5, 0, 0.5), (0, 0, 0, 1), rospy.Time.now(), 'cam1', 'ego_car')
        #self.br.sendTransform((1.7, 1.0, 0.5), (0, 0, 0.383, 0.924), rospy.Time.now(), 'cam2', 'ego_car')
        #self.br.sendTransform((1.7, -1.0, 0.5), (0, 0, -0.383, 0.924), rospy.Time.now(), 'cam3', 'ego_car')

        #self.pub_cam1.publish(square_pyramid_marker('cam1'))
        #self.pub_cam2.publish(square_pyramid_marker('cam2'))
        #self.pub_cam3.publish(square_pyramid_marker('cam3'))

        # mingu
        self.ego_car.header.stamp = rospy.Time.now()

        self.pub_ego_car.publish(self.ego_car)
        self.pub_enu_pose.publish(CS.position.x, CS.position.y, CS.yawRate)