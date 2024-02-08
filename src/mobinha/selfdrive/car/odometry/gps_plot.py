import rospy
from novatel_oem7_msgs.msg import INSPVA
import pymap3d
import time
import matplotlib.pyplot as plt

class GPSPlot:
    def __init__(self):
        rospy.Subscriber('/novatel/oem7/inspva',INSPVA, self.novatel_cb)
        
        self.lat = 0
        self.lon = 0

        self.init_lat = 0
        self.init_lon = 0

    def novatel_cb(self, msg):
        if self.init_lat > 37:
            self.lat = msg.latitude
        else:
            self.init_lat = msg.latitude

        if self.init_lon > 126:
            self.lon = msg.longitude
        else:
            self.init_lon = msg.longitude

    def gps_plot(self):
        x, y, _ = pymap3d.geodetic2enu(self.lat, self.lon, 0, self.init_lat, self.init_lon, 0)
        plt.scatter(x, y, color = 'blue', s=10)
        plt.pause(0.1)


if __name__=='__main__':
    rospy.init_node('gps_plot')

    g = GPSPlot()
    time.sleep(1)
    while not rospy.is_shutdown():
        g.gps_plot()

    plt.show()
    rospy.spin()
    