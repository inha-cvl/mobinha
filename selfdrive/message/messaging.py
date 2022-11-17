#!/usr/bin/env python3

from message.car_message import control, param, state, event
import rospy
from std_msgs.msg import Int16


class ParamMaster:
    def __init__(self):
        self.CP = param.Param()


class ControlMaster:
    def __init__(self):
        self.CC = control.Control()

    def update(self):
        car_control = self.CC._asdict()
        self.CC = self.CC._make(car_control.values())


class StateMaster:
    def __init__(self):
        rospy.init_node('state_master', anonymous=False)
        self.sub_test = rospy.Subscriber('/test', Int16, self.test_callback)
        self.test = 0
        self.CS = state.State()

    def test_callback(self, msg):
        self.test = msg.data

    def update(self):
        car_state = self.CS._asdict()
        car_state["number"] = self.test
        self.CS = self.CS._make(car_state.values())
