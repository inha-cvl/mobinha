from std_msgs.msg import Float32
import rospy


if __name__ == '__main__':
    rospy.init_node('Long', anonymous=False)
    pub_v = rospy.Publisher('/target_v', Float32, queue_size=1)

    while not rospy.is_shutdown():
        v = input('')
        pub_v.publish(Float32(v))
    
