#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from morai_msgs.msg import EgoVehicleStatus,CtrlCmd
import tf
from math import pi, tan

from std_msgs.msg import Float32

class gen_planner():
    def __init__(self):
        rospy.init_node('gen_planner', anonymous=True)

        #publisher
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        ctrl_msg= CtrlCmd()
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber('/wheel_angle', Float32, self.wheel_angle_cb)
        rospy.Subscriber('/accel_brake', Float32, self.accel_brake_cb)

        #def
        self.is_status=False ## 차량 상태 점검
        self.wheel_angle = 0
        self.accel_brake = None

        #time var
        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            if self.is_status :
        
                ctrl_msg.steering = self.wheel_angle if self.wheel_angle is not None else 0
                control_input = self.accel_brake if self.accel_brake is not None else -1

                if control_input > 0 :
                    ctrl_msg.accel= control_input
                    ctrl_msg.brake= 0
                else :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -control_input

                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력
        
                rate.sleep()
    
    def wheel_angle_cb(self, msg):
        # self.wheel_angle += self.velocity*0.1 * tan(msg.data) / 2.65
        # self.wheel_angle = (self.wheel_angle + pi)%(2*pi)-pi
        self.wheel_angle = msg.data * 180 / pi
        
    def accel_brake_cb(self, msg):
        self.accel_brake = msg.data

    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.velocity = data.velocity.x
        self.is_status=True
    
if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass
