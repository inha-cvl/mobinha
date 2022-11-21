import sys
import signal
import time

import rospy
from rviz import bindings as rviz
from std_msgs.msg import String, Float32

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic

from selfdrive.message.messaging import *


form_class = uic.loadUiType("forms/main.ui")[0]

MPH_TO_KPH = 3.6


class MainWindow(QMainWindow, form_class):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)

        self.car_name = str(self.car_name_combo_box.currentText())
        self.CP = None
        self.finish_cnt = 0

        self.sub_wheel_angle = rospy.Subscriber(
            '/wheel_angle', Float32, self.wheel_angle_cb)
        self.sub_accel_brake = rospy.Subscriber(
            '/accel_brake', Float32, self.accel_brake_cb)

        self.state = 'WAITING'
        # 0:wait, 1:start, 2:initialize
        self.pub_state = rospy.Publisher('/state', String, queue_size=1)

        self.initialize()
        self.connection_setting()

    def initialize(self):
        car_class = getattr(sys.modules[__name__], self.car_name)
        rospy.set_param('car_name', self.car_name)
        self.CP = car_class.CP

        # setting rviz
        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.setSplashPath("")
        self.rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "main.rviz")
        self.rviz_frame.load(config)
        self.rviz_frame.setMenuBar(None)
        self.rviz_frame.setStatusBar(None)

        self.manager = self.rviz_frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)
        self.rviz_layout.addWidget(self.rviz_frame)

    def connection_setting(self):
        self.start_button.clicked.connect(self.start_button_clicked)
        self.initialize_button.clicked.connect(self.initialize_button_clicked)
        self.finish_button.clicked.connect(self.finish_button_clicked)
        self.car_name_combo_box.currentIndexChanged.connect(
            self.car_name_changed)

    def car_name_changed(self, car_name):
        self.initialize()

    def wheel_angle_cb(self, msg):
        self.label_target_yaw.setText(str(round(msg.data*10, 5))+" deg")

    def accel_brake_cb(self, msg):
        self.label_target_v.setText(
            str(float(round(msg.data*MPH_TO_KPH)))+" km/h")

    def start_button_clicked(self):
        sm = StateMaster(CP)
        self.state = 'START'

        while True:
            self.pub_state.publish(String(self.state))
            time.sleep(0.1)
            if self.state == 'START':
                sm.update()
                self.display(sm.CS)
            elif self.state == 'FINISH':
                self.finish_cnt += 1
                if(self.finish_cnt == 30):
                    print("[Visualize Process] Over")
                    sys.exit(0)
            QApplication.processEvents()

    def initialize_button_clicked(self):
        self.state = 'INITIALIZE'

    def finish_button_clicked(self):
        self.state = 'FINISH'

    def display(self, CS):
        self.label_vehicle_vel.setText(
            str(float(round(CS.vEgo*MPH_TO_KPH)))+" km/h")
        self.label_vehicle_yaw.setText(str(round(CS.yawRate, 5))+" deg")


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    print("[Visualize Process] Start")
    rospy.init_node('Visualize', anonymous=False)

    try:
        app = QApplication(sys.argv)
        mainWindow = MainWindow()
        mainWindow.show()
        app.exec_()

    except Exception as e:
        print("[Visualize Error]", e)

    except KeyboardInterrupt:
        print("[Visualize Process] Force Quit")
        sys.exit(0)


if __name__ == "__main__":
    main()
