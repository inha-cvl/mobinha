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
            self.red_3:         7,
            self.yellow_3:      8,
            self.green_3:       13,

            self.red_4:         7,     
            self.yellow_4:      8,
            self.green_4:       13,
            
            self.red_yellow_4:  10,
            self.red_arrow_4:   9,
            self.arrow_green_4: 11
        }
        self.initialize()

    def initialize(self):
        for btn, idx in self.tl_list.items():
            btn.clicked.connect(lambda _=False, b=btn, i=idx: self.tl_button_clicked(b, i))


        self.stop_button.clicked.connect(self.stop_button_clicked)
        self.tl_simulator_timer.setInterval(100)
        self.tl_simulator_timer.timeout.connect(self.publish_test_tl_bbox)
        self.tl_simulator_timer.start()
    
    def tl_button_clicked(self, pressed_btn, idx):
        if self.tl_type == idx:      # 토글 역할
            self.stop_button_clicked()
            return

        self.tl_type = idx
        if not self.tl_simulator_timer.isActive():
            self.tl_simulator_timer.start()

        # 자신을 제외한 나머지 모두 비활성화
        for btn in self.tl_list.keys():
            btn.setDisabled(btn is not pressed_btn)

    def stop_button_clicked(self):
        self.tl_type = 0

  
        for btn in self.tl_list.keys():      
            btn.setEnabled(True)    
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