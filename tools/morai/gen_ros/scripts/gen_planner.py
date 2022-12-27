#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import numpy as np
from nav_msgs.msg import Path,Odometry
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader, findLocalPath,purePursuit,pidController,velocityPlanning,vaildObject,cruiseControl
import tf
from math import pi

class gen_planner():
    def __init__(self):
        rospy.init_node('gen_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        self.traffic_control=arg[2]

        path_reader=pathReader('gen_ros') ## 경로 파일의 위치
        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        traffic_pub = rospy.Publisher("/SetTrafficLight",SetTrafficLight,queue_size=1) ## TrafficLight Control
        
        ctrl_msg= CtrlCmd()
        odom_pub= rospy.Publisher('/odom',Odometry,queue_size=1)
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.getTL_callback) ## TrafficLight information Subscriber
        
        #def
        self.is_status=False ## 차량 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.is_traffic=False ## 신호등 상태 점검
        self.traffic_info = [[58.50 , 1180.41, 'C119BS010001'], ## TrafficLight information
                             [85.61,1227.88 ,'C119BS010021'],
                             [136.58,1351.98 ,'C119BS010025'],
                             [141.02,1458.27 ,'C119BS010028'],
                             [139.39,1596.44 ,'C119BS010033']]




        self.global_path=path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름
        

        #class
        pure_pursuit=purePursuit() ## purePursuit import
        pid=pidController()
        self.cc=cruiseControl(0.5,1) ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.vo=vaildObject(self.traffic_info) ## 장애물 유무 확인 (TrafficLight)
        vel_planner=velocityPlanning(60/3.6,0.15) ## 속도 계획
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,100)


        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            if self.is_status==True and self.is_obj ==True:
                
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg)
                self.vo.get_object(self.object_num,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3])
                global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])
                if self.is_traffic == True: ## 신호등 상태 점검 
                    if self.traffic_control == "True": ## 신호등 신호 초록색으로 변경(True) or (False)
                        self.tl_msg.trafficLightStatus=16 

                        ## traffic_control ##
                        self.set_traffic_data= SetTrafficLight()
                        self.set_traffic_data.trafficLightIndex = self.tl_msg.trafficLightIndex
                        self.set_traffic_data.trafficLightStatus = 16 ## 16 = Green light
                        ## traffic_control ##

                        traffic_pub.publish(self.set_traffic_data) ## 차량 신호 Green Light 변경
                    self.cc.checkObject(local_path,global_obj,local_obj,[self.tl_msg.trafficLightIndex,self.tl_msg.trafficLightStatus]) 
                else :
                    self.cc.checkObject(local_path,global_obj,local_obj)
                # pure pursuit control
                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
                ctrl_msg.steering=-pure_pursuit.steering_angle()/180*pi
        

                cc_vel = self.cc.acc(local_obj,self.status_msg.velocity.x,vel_profile[self.current_waypoint],self.status_msg.position) ## advanced cruise control 적용한 속도 계획
                target_velocity = cc_vel


                control_input=pid.pid(target_velocity,self.status_msg.velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)


                if control_input > 0 :
                    ctrl_msg.accel= control_input
                    ctrl_msg.brake= 0
                else :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -control_input


                local_path_pub.publish(local_path) ## Local Path 출력
                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력
                odom_pub.publish(self.makeOdomMsg())
                
            
                if count==30 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    count=0
                count+=1
                rate.sleep()



    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True


    def makeOdomMsg(self):
        odom=Odometry()
        odom.header.frame_id='map'
        odom.child_frame_id='gps'

        quaternion = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(self.status_msg.heading))

        odom.pose.pose.position.x=self.status_msg.position.x
        odom.pose.pose.position.y=self.status_msg.position.y
        odom.pose.pose.position.z=self.status_msg.position.z
        odom.pose.pose.orientation.x=quaternion[0]
        odom.pose.pose.orientation.y=quaternion[1]
        odom.pose.pose.orientation.z=quaternion[2]
        odom.pose.pose.orientation.w=quaternion[3]


        return odom

    def objectInfoCB(self,data): ## Object information Subscriber
        self.object_num=data.num_of_npcs+data.num_of_obstacle+data.num_of_pedestrian
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]
        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        for num in range(data.num_of_obstacle) :
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity.x)

        for num in range(data.num_of_pedestrian) :
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            object_pose_y.append(data.pedestrian_list[num].position.y)
            object_velocity.append(data.pedestrian_list[num].velocity.x)

        self.object_info=[object_type,object_pose_x,object_pose_y,object_velocity]
        self.is_obj=True

    def getTL_callback(self,msg): ## TrafficLight information Subscriber
        self.is_traffic=True
        self.tl_msg=GetTrafficLightStatus()
        self.tl_msg=msg
    
if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass
