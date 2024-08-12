import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":
    rospy.init_node("virtual_distance_pub")
    virtual_distance_pub = rospy.Publisher('/current_distance', Float32, queue_size=1)
    cmd = None
    msg = Float32()
    while not rospy.is_shutdown():
        cmd = float(input(f'Insert current distance(now:{cmd}): '))
        msg.data = cmd
        virtual_distance_pub.publish(cmd)
        print(f"published {msg.data:.1f}")

