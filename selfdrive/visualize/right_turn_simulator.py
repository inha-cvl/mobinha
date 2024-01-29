from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic
import os
import rospy
from std_msgs.msg import Int8MultiArray

dir_path = str(os.path.dirname(os.path.realpath(__file__)))
form_class = uic.loadUiType(dir_path+"/forms/right_turn_simulator.ui")[0]



class RTISimulator(QMainWindow, form_class):
    def __init__(self, parent=None):
        super(RTISimulator, self).__init__(parent)
        self.setupUi(self)
        self.right_turn_simulator_timer = QTimer()
        self.right_turn_type = [1,1]

        #ROS Topic set
        self.pub_right_turn_situation = rospy.Publisher('/mobinha/planning/right_turn_situation', Int8MultiArray, queue_size=1)
        self.situation_list = {
            'c1p0': self.c1p0,
            'c0p1': self.c0p1,
            'c1p1': self.c1p1,
            'c0p0': self.c0p0
        }
        self.initialize()

    def initialize(self):
        for key,value in self.situation_list.items():
            value.clicked.connect(lambda state,  button_key=key: self.situation_button_clicked(button_key))
        # self.stop_button.clicked.connect(self.stop_button_clicked)
        self.right_turn_simulator_timer.setInterval(100)
        self.right_turn_simulator_timer.timeout.connect(self.publish_right_turn_situation)
        self.right_turn_simulator_timer.start()
    
    def situation_button_clicked(self, button_key):
        if button_key == 'c1p0':
            self.right_turn_type = [1, 0]
        elif button_key == 'c0p1':
            self.right_turn_type = [0, 1]
        elif button_key == 'c1p1':
            self.right_turn_type = [1, 1]
        elif button_key == 'c0p0':
            self.right_turn_type = [0, 0]

    def publish_right_turn_situation(self):
        situation = Int8MultiArray()
        situation.data = self.right_turn_type
        self.pub_right_turn_situation.publish(situation)