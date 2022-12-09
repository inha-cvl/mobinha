#!/usr/bin/python
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from selfdrive.visualize.viz_utils import *
from libs.purepursuit import PurePursuit
from libs.pid import PID
import rospy

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


class Controller:

    def __init__(self, CP):
        self.pid = PID(CP.longitudinalTuning)
        self.purepursuit = PurePursuit(CP)

        self.target_v = 0.0
        self.local_path = None

        self.sub_local_path = rospy.Subscriber(
            '/local_path', Marker, self.local_path_cb)
        self.sub_target_v = rospy.Subscriber(
            '/target_v', Float32, self.target_v_cb)

        self.pub_wheel_angle = rospy.Publisher(
            '/wheel_angle', Float32, queue_size=1)
        self.pub_accel_brake = rospy.Publisher(
            '/accel_brake', Float32, queue_size=1)
        self.pub_lah = rospy.Publisher(
            '/look_ahead', Marker, queue_size=1, latch=True)

    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def target_v_cb(self, msg):
        self.target_v = msg.data

    def run(self, sm):
        CS = sm.CS
        if CS.yawRate != 0.0:
            if self.local_path is None:
                wheel_angle = 0.0
            else:
                wheel_angle, lah_pt = self.purepursuit.run(
                    CS.position.x, CS.position.y, CS.yawRate, CS.vEgo, self.local_path)
                lah_viz = LookAheadViz(lah_pt)
                self.pub_lah.publish(lah_viz)

            accel_brake = self.pid.run(self.target_v, CS.vEgo)

            self.pub_wheel_angle.publish(Float32(wheel_angle))
            self.pub_accel_brake.publish(Float32(accel_brake))
