import rospy
from std_msgs.msg import Int8, Float32

from selfdrive.message.car_message import car_control

CC = car_control.CarControl()


class ControlMaster:
    def __init__(self):
        self.CC = CC

        self.accel = 0.0
        self.brake = 0.0
        self.steer = 0.0
        # disable, enable, lat, lng
        self.can_cmd = {"disable": False, "enable": False,
                        "latActive": False, "longActive": False}

        rospy.Subscriber('/mobinha/control/wheel_angle',
                         Float32, self.wheel_angle_cb)
        rospy.Subscriber('/mobinha/control/accel_brake',
                         Float32, self.accel_brake_cb)
        rospy.Subscriber('/mobinha/visualize/can_cmd', Int8, self.can_cmd_cb)

    def wheel_angle_cb(self, msg):
        self.steer = msg.data

    def accel_brake_cb(self, msg):
        if msg.data > 0:
            self.accel = msg.data
            self.brake = 0
        else:
            self.accel = 0
            self.brake = -msg.data

    def can_cmd_cb(self, msg):
        for idx, cmd in enumerate(self.can_cmd):
            if idx == msg.data:
                self.can_cmd[cmd] = True
            else:
                self.can_cmd[cmd] = False

    def update(self):
        car_control = self.CC._asdict()
        car_control_cancmd = car_control["canCmd"]._asdict()
        car_control_cancmd = self.can_cmd
        car_control["canCmd"] = self.CC.canCmd._make(
            car_control_cancmd.values())
        car_control_actuators = car_control["actuators"]._asdict()
        car_control_actuators["brake"] = self.brake
        car_control_actuators["steer"] = self.steer
        car_control_actuators["accel"] = self.accel
        car_control["actuators"] = self.CC.actuators._make(
            car_control_actuators.values())
        self.CC = self.CC._make(car_control.values())
