#!/usr/bin/python
import time
import argparse
import os
import pickle
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import numpy as np

import rospy
import math
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray, Int32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from jsk_recognition_msgs.msg import PolygonArray

dir_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(dir_path)

from config.config import Config
from lanelet_map import LaneletMap
from tile_map import TileMap
from micro_lanelet_graph import MicroLaneletGraph
from odom import Odom
from planner_utils import * 
from fetch_light import fetch_light, classify_light
from viz import *

class Planner:
    def __init__(self):
        rospy.init_node('planner', anonymous=False)

        self.config = Config()
        self.lmap = LaneletMap(self.config.map_path)
        self.tmap = TileMap(self.lmap.lanelets, self.config.tile_size)
        self.mlgraph = MicroLaneletGraph(self.lmap, self.config.cut_dist) ###############################
        # for id_ in ['72', '44']:
        #     for to_id in self.mlgraph.graph[id_]:
        #         self.mlgraph.graph[id_][to_id] = float('inf')
        ###############################
        self.odom = Odom()

        self.traffic_lights = [0, 36001, 0, 36001, 0, 36001]
        self.waypoint = None
        self.localpoint = None
        self.sub_waypoint = None
        self.final_path = None
        self.local_path = []
        self.local_calc = True
        self.local_state = 0 #0 : Maintain, 1 : Switching, 2 : Switched
        self.local_route = 0
        self.current_route = 1
        self.overtake = False
        self.qit = None
        self.construction_sites = []
        self.get_goal = False
        self.lidar_object = []
        self.camera_object = [0, 0]
        self.kdtree = False
        self.pub_goal = rospy.Publisher('/goal', Marker, queue_size=1, latch=True) 
        self.pub_shortest_path = rospy.Publisher('/shortest_path', MarkerArray, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher('/local_path', Marker, queue_size=1, latch=True)
        self.pub_final_path = rospy.Publisher('/final_path', Marker, queue_size=1, latch=True)
        self.pub_target_v = rospy.Publisher('/target_v', Float32, queue_size=1, latch=True)
        self.pub_can_cmd = rospy.Publisher('/can_cmd', Int16MultiArray, queue_size=1)
        self.pub_micro_lanelet_graph = rospy.Publisher('/micro_lanelet_graph', MarkerArray, queue_size=1, latch=True)

        rospy.Subscriber('/odom', Float32MultiArray, self.odom_cb)
        rospy.Subscriber('/spat_msg', Float32MultiArray, self.v2x_cb)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/construction_sites', MarkerArray, self.construction_sites_cb)
        rospy.Subscriber('/camera_ob', Marker, self.camera_cb)
        rospy.Subscriber('/lidar_ob', Marker, self.lidar_cb)
        rospy.Subscriber('/overtake', Bool, self.overtake_cb)

    def goal_cb(self, msg):
        self.goal_pt = [msg.pose.position.x, msg.pose.position.y]
        self.get_goal = True

    def odom_cb(self, msg):
        self.odom.set(msg.data)

    def overtake_cb(self, msg):
        self.overtake = True

    def construction_sites_cb(self, msg):
        self.construction_sites = []

        for marker in msg.markers:
            pts = []
            for pt in marker.points:
                pts.append((pt.x, pt.y))
            self.construction_sites.append(pts)

    def v2x_cb(self, msg):
        self.traffic_lights = msg.data

    def lidar_cb(self, msg): #Get lidar object and convert into frenet (s,d)
        if not self.kdtree:
            return
        objects = []
        for obj in msg.points:
            s,d = self.object2frenet(obj)
            s = s - 3.5
            objects.append([s,d])
        self.lidar_object = objects
        #print("Lidar : {}".format(s))

    def camera_cb(self, msg): #Get camera object and convert into frenet (s,d)
        if not self.kdtree:
            return
        objects = []
        s,d = self.object2frenet(msg.pose.position)
        self.camera_object = [s,d]
        #print("Camera : {}".format(s))

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def object2frenet(self, obj): #Array in Frenet Array Out
        target = [obj.x, obj.y]
        point = self.localpoint.query(target, 1)[1]
        if(point == 0):
            return 0, 1000
        # point = self.waypoint.query(target, 1)[1]
        # point = self.sub_waypoint.query(target, 1)[1]
        wp = self.local_path
        ob = np.array(target)

        n_x = wp[point][0] - wp[point-1][0]
        n_y = wp[point][1] - wp[point-1][1]
        x_x = target[0] - wp[point-1][0]
        x_y = target[1] - wp[point-1][1]

        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y

        frenet_d = self.distance(x_x,x_y,proj_x,proj_y)

        center_x = 900 - wp[point-1][0]
        center_y = -500 - wp[point-1][1]
        distToPos = self.distance(center_x, center_y, x_x, x_y)
        distToRef = self.distance(center_x, center_y, proj_x, proj_y)

        if(distToPos > distToRef):
            frenet_d *= -1

        return int(point/2),frenet_d

    def getXY(self, s, d):
        qit = self.qit
	
        x,y = qit.calc_position(s)
        yaw = qit.calc_yaw(s)

        x = x + d*np.cos(yaw)
        y = y + d*np.sin(yaw)

        print(x,y)

    def run(self):
        rate = rospy.Rate(20) # 10hz

        turn_sig = 0

        start_time = None
        state = 'Ready'
        shortest_path = None
        g_id, g_idx = None, None
        start_path = []
        final_path = []
        max_v = []
        last_s = None
        precision = 0.5

        start_sequence = True
        last_s = 0

        ttt = 0
        iii = 0

        # velocity plan parameters
        min_v = self.config.min_v
        ref_v = self.config.ref_v

        st_param = {'s_min':-20.0, 's_max':70.0, 't_max':8.0, 'dt':0.3, 'dt_exp':1.5}

        plt.ion()

        fig = plt.figure(figsize=(5,5), dpi=80)
        ax = fig.add_subplot(111)
        ax.set_xlim([0.0, st_param['t_max']])
        ax.set_xticks([i for i in np.arange(0, int(st_param['t_max']), 0.5)])
        ax.grid(color='#BDBDBD', linestyle='-', linewidth=2, )
        drawn = None

        wolf = 0
        alpha_path = []

        # with open('weekdays/weekdays_1121.pkl', 'rb') as f:
            # main_path = pickle.load(f)

        # with open('weekdays/start_straight.pkl', 'rb') as f:
            # start_path = pickle.load(f)

        with open('weekdays/start_right.pkl', 'rb') as f:
            start_path = pickle.load(f)

        with open('weekdays/local/inside.pkl', 'rb') as f:
            main_path = pickle.load(f)

        with open('weekdays/local/outside.pkl', 'rb') as f:
            sub_path = pickle.load(f)

        while not rospy.is_shutdown():
            if state == 'Ready':
                if self.get_goal:
                    if start_sequence:
                        rospy.loginfo('Start Sequence')
                        final_path, max_v, last_s, qit = interpolate(start_path, precision, min_v, ref_v)
                        sub_final_path, sub_max_v, sub_last_s, qit = interpolate(sub_path, precision, min_v, ref_v) 
                        alpha_path = final_path
                        self.final_path = final_path
			self.qit = qit
                        self.waypoint = KDTree(final_path)
                        self.sub_waypoint = KDTree(sub_final_path)
                        self.localpoint = KDTree(final_path)
                        final_path_viz = FinalPathViz(final_path)
                        self.pub_final_path.publish(final_path_viz)
                        self.kdtree = True
                        #For overtake
                        alpha_path = final_path
                    else:
                        rospy.loginfo('Lift off! Good luck!')
                        self.kdtree = False
                        final_path, max_v, last_s, qit = interpolate(main_path, precision, min_v, ref_v)
                        sub_final_path, sub_max_v, sub_last_s, qit = interpolate(sub_path, precision, min_v, ref_v) 
                        last_s = 5000
                        self.final_path = final_path
                        self.waypoint = KDTree(final_path)
                        self.sub_waypoint = KDTree(sub_final_path)
                        self.localpoint = KDTree(final_path)
                        final_path_viz = FinalPathViz(final_path)
                        self.pub_final_path.publish(final_path_viz)
                        self.kdtree = True
                        #For overtake
                        alpha_path = final_path
                        
                    start_time = time.time()
                    # self.get_goal = False
                    state = 'MoveToGoal'

            elif state == 'MoveToGoal':
                cte, idx = calc_cte_and_idx(final_path, (self.odom.x, self.odom.y))
                s = idx * precision

                local_path = []

                ### OVERTAKE
                a_idx = self.sub_waypoint.query((self.odom.x, self.odom.y),1)[1]

                speed_variable_in = int(15 * (self.odom.v * 3.6)/50)
                speed_variable_out = int(25 * (self.odom.v * 3.6)/50)

                if self.local_state == 0 and self.overtake:
                    self.local_state = 1

                if self.local_state == 1:
                    if len(final_path) - idx > 35:
                        stitch = self.sub_waypoint.query(final_path[idx+10+speed_variable_in],1)[1]
                        alpha_path = final_path[:idx + 10 + speed_variable_in] + sub_final_path[stitch + 15 + speed_variable_out:] 
                        self.local_state = 2
                        self.overtake = False
                        self.current_route = 2

                elif self.local_state == 2 and self.overtake:
                    if len(sub_final_path) - a_idx > 35:
                        stitch = self.waypoint.query(sub_final_path[a_idx+10+speed_variable_in],1)[1]
                        alpha_path = sub_final_path[:a_idx + 10 + speed_variable_in] + final_path[stitch + 15 + speed_variable_out:]
                        self.local_state = 0
                        self.overtake = False
                        self.current_route = 1

                cte, fidx = calc_cte_and_idx(alpha_path, (self.odom.x, self.odom.y))

                if len(alpha_path) - fidx < 40:
                    if self.current_route == 1:
                        alpha_path = final_path
                    elif self.current_route == 2:
                        alpha_path = sub_final_path

                if(len(alpha_path) - fidx > max(61,self.odom.v * 360/50) and self.local_calc):
                    for i in range(0, max(61, int(self.odom.v * 360/50)), 5):
                        local_path.append(alpha_path[fidx + int(i)])
                    self.local_path, local_max, local_last_s, qit = interpolate(local_path, precision, min_v, ref_v)
                    self.localpoint = KDTree(self.local_path)
                    self.local_calc = False
                elif self.local_calc:
                    s_x = self.odom.x + 20 * math.cos(math.radians(self.odom.yaw))
                    s_y = self.odom.y + 20 * math.sin(math.radians(self.odom.yaw))
                    local_path.append((self.odom.x, self.odom.y))
                    local_path.append((s_x, s_y))
                    self.local_path, local_max, local_last_s, qit = interpolate(local_path, precision, min_v, ref_v)
                    self.localpoint = KDTree(self.local_path)

                local_path_viz = LocalPathViz(self.local_path)
                self.pub_local_path.publish(local_path_viz)

                light_s, light_idx = fetch_light(s, start_sequence, self.current_route)
                light_state, light_time = classify_light(light_idx, self.traffic_lights)
                if start_sequence == False:
                    light_s = light_s - s

                print(s)

                if len(self.local_path) > 1:
                    l_cte, l_idx = calc_cte_and_idx(self.local_path, (self.odom.x, self.odom.y))
                    local_s = l_idx * precision  
                    if(len(self.local_path) - l_idx < 80 + 90 * self.odom.v*3.6/50):
                        self.local_calc = True

                self.lidar_object.append(self.camera_object)

                #If in start sequence use global s for searching 
                if start_sequence == True:
                    local_s = s

                target_v, drawn = velocity_plan(ax, drawn, st_param, ref_v, last_s, local_s, local_max, self.odom.v, self.odom.a, light_s, light_state, light_time, self.lidar_object)

                # Speed limit for zone
                target_v = min(target_v, 40/3.6) if s < 295 else target_v
                self.pub_target_v.publish(Float32(min(target_v, 50/3.6)))

                fig.canvas.draw()
                fig.canvas.flush_events()
                if last_s - s < 15.0 and self.odom.v < 1.0 / 3.6:
                    final_path = []
                    rospy.loginfo('Arrived! Running Time: %s'%(time.time() - start_time))
                    state = 'Ready'
                    start_sequence = False

            can_cmd = Int16MultiArray()
            # EPS_En, Override_Ignore, EPS_Speed, ACC_En, AEB_En, Gear_Change, Turn_Sig_En
            can_cmd.data = [self.config.eps_en, 0, self.config.eps_speed, self.config.acc_en, 0, 0, turn_sig]
            self.pub_can_cmd.publish(can_cmd)

            rate.sleep()


if __name__ == "__main__":
    planner = Planner()
    planner.run()

