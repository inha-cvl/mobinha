from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic
import os
import rospy
from geometry_msgs.msg import PoseArray, Pose

dir_path = str(os.path.dirname(os.path.realpath(__file__)))
form_class = uic.loadUiType(dir_path+"/forms/tl_simulator.ui")[0]

'''
[13]Green3 [7]Red3 [8]Yellow3
[13]Green4 [7]Red4 [8]Yellow4
[9]Red Arrow4 [10]Red Yellow4 [11]Arrow Green4
'''

class TLSimulator(QMainWindow, form_class):
    def __init__(self, parent=None):
        super(TLSimulator, self).__init__(parent)
        self.setupUi(self)
        self.tl_simulator_timer = QTimer()
        self.tl_type = 0

        #ROS Topic set
        self.pub_bounding_box = rospy.Publisher('/mobinha/perception/camera/bounding_box', PoseArray, queue_size=1)
        self.tl_list = {
            7:self.red_3,
            8:self.yellow_3,
            13:self.green_3,
            7:self.red_4,
            8:self.yellow_4,
            13:self.green_4,
            10:self.red_yellow_4,
            9:self.red_arrow_4,
            11:self.arrow_green_4
        }
        self.initialize()

    def initialize(self):
        for key,value in self.tl_list.items():
            value.clicked.connect(lambda state,  idx=key: self.tl_button_clicked(idx))
        self.stop_button.clicked.connect(self.stop_button_clicked)
        self.tl_simulator_timer.setInterval(100)
        self.tl_simulator_timer.timeout.connect(self.publish_test_tl_bbox)
        self.tl_simulator_timer.start()
    
    def tl_button_clicked(self, idx):
        if self.tl_type == idx:
            self.stop_button_clicked()
            return
        self.tl_type = idx
        if not self.tl_simulator_timer.isActive():
            self.tl_simulator_timer.start()
        for key, value in self.tl_list.items():
            if key != idx:
                value.setDisabled(True)
        
    def stop_button_clicked(self):
        self.tl_type = 0
        for key, value in self.tl_list.items():
            value.setEnabled(True)
        if self.tl_simulator_timer.isActive():
            self.tl_simulator_timer.stop()
            self.publish_test_tl_bbox()

    def publish_test_tl_bbox(self):
        bounding_box = PoseArray()
        pose = Pose()
        pose.position.x = self.tl_type
        pose.position.y = 0.8
        pose.position.z = 1.0
        bounding_box.poses.append(pose)
        self.pub_bounding_box.publish(bounding_box)