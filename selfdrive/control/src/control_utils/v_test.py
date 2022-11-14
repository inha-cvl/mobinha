from std_msgs.msg import Float32
import rospy
rospy.init_node('tmp_v', anonymous=False)
pub= rospy.Publisher('/tmp_target_v', Float32, queue_size=1)
while True:
    target = input('put vel input: ')
    if target == '-1':
        exit(0)
    target = float(target)
    pub.publish(Float32(target))
