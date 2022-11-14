from std_msgs.msg import Float32
import rospy

def print_V(msg):
    print(msg.data*3.6)

if __name__ == '__main__':
    rospy.init_node('Test_v', anonymous=False)
    rospy.Subscriber('/car_v', Float32, print_V)
    rospy.spin()

    
