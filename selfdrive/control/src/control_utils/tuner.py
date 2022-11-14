from std_msgs.msg import Float32
import rospy
rospy.init_node('tuner', anonymous=False)
v_pub= rospy.Publisher('/tmp_target_v', Float32, queue_size=1)
lfc_pub= rospy.Publisher('/tmp_target_lfc', Float32, queue_size=1)
k_pub= rospy.Publisher('/tmp_target_k', Float32, queue_size=1)
print("tar_v==target velocity in km/h")
print("lfc==lookahead default(offset)")
print("k==lookahead gain for current velocity")
print("cur_v==current velocity")
print("lookahead = lfc + k*cur_v")
print("put 'exit' to exit")
while True:
    target = input('put tar_v,lfc,k: ').split(',')
    if target[0] == 'exit':
        exit(0)
    
    tar_v = float(target[0]) / 3.6
    v_pub.publish(Float32(tar_v))
    lfc = float(target[1])
    lfc_pub.publish(Float32(lfc))
    k = float(target[2])
    k_pub.publish(Float32(k))

    print(f"Set tar_v={tar_v}, lfc={lfc}, k={k}")