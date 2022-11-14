from std_msgs.msg import Int16MultiArray
import rospy
rospy.init_node('tmp_state', anonymous=False)
pub= rospy.Publisher('/stage1_state', Int16MultiArray, queue_size=1)

state = Int16MultiArray()
state.data = []
while True:
    target = input('put state input: ')
    if target == '-1':
        exit(0)
    state.data = [int(target), 1, 0, 0]
    pub.publish(state)