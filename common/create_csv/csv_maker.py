import rospy
import math
from sbg_driver.msg import SbgEkfNav, SbgEkfEuler


class INS():
    def __init__(self, filename):
        self.f_ekf = open('./{}.csv'.format(filename), "w")
        self.ekf_pos = [0, 0, 0]
        self.gps_pos = [0, 0, 0]
        self.thdt = 0
        self.hdt = 0
        self.acc = 0
        self.rtk = 0
        self.extractor()

    def nav(self, data):
        x, y, z = data.latitude, data.longitude, data.altitude
        self.f_ekf.write('{},{},{},{}\n'.format(x, y, z, self.hdt))

    def head(self, data):
        # pass
        yaw = math.degrees(data.angle.z)
        self.hdt = 90 - yaw if (yaw >= -90 and yaw <= 180) else -270 - yaw

    def extractor(self):
        rospy.init_node('extractor', anonymous=True)

        rospy.Subscriber('sbg/ekf_nav', SbgEkfNav, self.nav)
        rospy.Subscriber('sbg/ekf_euler', SbgEkfEuler, self.head)

        rospy.spin()


if __name__ == '__main__':
    filename = input('enter filename : ')
    alpha = INS(filename)
