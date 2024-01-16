from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic
import os
import rospy
from std_msgs.msg import Int8

dir_path = str(os.path.dirname(os.path.realpath(__file__)))
form_class = uic.loadUiType(dir_path+"/forms/blinker_simulator.ui")[0]


class BlinkerSimulator(QMainWindow, form_class):
    def __init__(self, parent=None):
        super(BlinkerSimulator, self).__init__(parent)
        self.setupUi(self)
        self.blinker_simulator_timer = QTimer()
        self.blinker = 0

        #ROS Topic set
        self.pub_blinker = rospy.Publisher('/turnsignal', Int8, queue_size=1)
        self.initialize()

    def initialize(self):
        # for key,value in self.tl_list.items():
        #     value.clicked.connect(lambda state,  idx=key: self.tl_button_clicked(idx))
        self.stop_button.clicked.connect(self.stop_button_clicked)
        self.left.clicked.connect(self.left_clicked)
        self.right.clicked.connect(self.right_clicked)
        self.blinker_simulator_timer.setInterval(100)
        self.blinker_simulator_timer.timeout.connect(self.publish_test_blinker)
        self.blinker_simulator_timer.start()
    
    def left_clicked(self):
        if self.blinker == 1:
            self.stop_button_clicked()
            return
        self.blinker = 1
        if not self.blinker_simulator_timer.isActive():
            self.blinker_simulator_timer.start()

    def right_clicked(self):
        if self.blinker == 2:
            self.stop_button_clicked()
            return
        self.blinker = 2
        if not self.blinker_simulator_timer.isActive():
            self.blinker_simulator_timer.start()

    def stop_button_clicked(self):
        self.blinker = 0
        # for key, value in self.tl_list.items():
        #     value.setEnabled(True)
        if self.blinker_simulator_timer.isActive():
            self.blinker_simulator_timer.stop()
            self.publish_test_blinker()

    def publish_test_blinker(self):
        blinker=Int8()
        blinker.data = self.blinker
        self.pub_blinker.publish(blinker)
        # bounding_box = PoseArray()
        # pose = Pose()
        # pose.position.x = self.tl_type
        # pose.position.y = 0.8
        # pose.position.z = 1.0
        # bounding_box.poses.append(pose)
        # self.pub_bounding_box.publish(bounding_box)