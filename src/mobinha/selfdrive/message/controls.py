import rospy
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Vector3
from selfdrive.message.car_message import car_control

CC = car_control.CarControl()


class ControlMaster:
    def __init__(self):
        self.CC = CC
        self.accerror = 0.0
        # disable, enable, lat, lng
        self.can_cmd = {"disable": False, "enable": False,"latActive": False, "longActive": False}
        self.target_actuators = {"steer":0.0, "accel":0.0, "brake":0.0}

        rospy.Subscriber('/mobinha/control/target_actuators', Vector3, self.target_actuators_cb)
        rospy.Subscriber('/mobinha/visualize/can_cmd', Int8, self.can_cmd_cb)
        rospy.Subscriber('/mobinha/control/accerror', Float32, self.accerror_cb)

    def target_actuators_cb(self, msg):
        self.target_actuators["steer"] = msg.x
        self.target_actuators["accel"] = msg.y
        self.target_actuators["brake"] = msg.z

    def can_cmd_cb(self, msg):
        for idx, cmd in enumerate(self.can_cmd):
            if idx == msg.data:
                self.can_cmd[cmd] = True
            else:
                self.can_cmd[cmd] = False

    def accerror_cb(self, msg):
        self.accerror = msg.data

    def update(self):
        car_control = self.CC._asdict()
        car_control["canCmd"] = self.CC.canCmd._make(self.can_cmd.values())
        car_control["actuators"] = self.CC.actuators._make(self.target_actuators.values())
        car_control_cruisecontrol = car_control["cruiseControl"]._asdict()
        car_control_cruisecontrol["accerror"] = self.accerror
        car_control["cruiseControl"] = self.CC.cruiseControl._make(car_control_cruisecontrol.values())
        self.CC = self.CC._make(car_control.values())
