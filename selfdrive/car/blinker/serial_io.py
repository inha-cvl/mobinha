import rospy
import serial
from std_msgs.msg import Int8

class SerialIO:
    def __init__(self, port='/dev/ttyUSB0'):
        #Variables
        self.state = 0 #0, 1, 2

        # Serial Connect
        self.ser = serial.Serial(port, 115200, timeout=.1)
        print("Serial_IO: Serial connecting to /dev/ttyUSB0...")

        # ROS 
        rospy.init_node("SerialIO", anonymous=False)
        rospy.Subscriber("/lane_change", Int8, self.roscb)
        print("Serial_IO: Initializing ROS node...")

    def roscb(self, msg):
        self.state = msg.data

    def serial_write(self, x):
        data = bytes(x, 'utf-8')
        self.ser.write(data)

    def run(self):
        self.serial_write(self.state)

if __name__ == '__main__':
    io = SerialIO(port='/dev/ttyUSB0')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        io.run()
        rate.sleep()    