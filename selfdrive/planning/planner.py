#!/usr/bin/python
import argparse
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from jsk_recognition_msgs.msg import PolygonArray

from lanelet_map import LaneletMap
from tile_map import TileMap
from micro_lanelet_graph import MicroLaneletGraph
from odom import Odom
from planner_utils import *
from viz import *
import copy

import os
g_path = os.path.dirname(os.path.abspath(__file__))

class Planner:
    def __init__(self, args):
        rospy.init_node('planner', anonymous=False)

        self.lmap = LaneletMap(args.map_path)
        self.tmap = TileMap(self.lmap.lanelets, args.tile_size)
        self.graph = MicroLaneletGraph(self.lmap, args.cut_dist).graph
        ################### TODO: delete! ################
        # for id_ in ['433', '72', '377', '374']:
        #     for l_id in self.graph[id_].keys():
        #         self.graph[id_][l_id] = float('inf')

        # self.const_site = [['401', 0, 20], ['421', 0, 20]]
        ##################################################

        self.odom = Odom()

        self.get_goal = False

        self.pub_goal = rospy.Publisher('/goal', Marker, queue_size=1, latch=True)
        self.pub_shortest_path = rospy.Publisher('/shortest_path', MarkerArray, queue_size=1, latch=True)
        self.pub_final_path = rospy.Publisher('/final_path', Marker, queue_size=1, latch=True)
        self.pub_target_v = rospy.Publisher('/target_v', Float32, queue_size=1, latch=True)
        self.pub_curvature = rospy.Publisher('/ego_curvature', Float32, queue_size=1, latch=True)
        self.pub_can_cmd = rospy.Publisher('/can_cmd', Int16MultiArray, queue_size=1)

        self.sub_odom = rospy.Subscriber('/odom', Float32MultiArray, self.odom_cb)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)

        self.pub_current_laneID = rospy.Publisher(
            '/current_LaneID', Int16MultiArray, queue_size=1)

    def goal_cb(self, msg):
        self.goal_pt = [msg.pose.position.x, msg.pose.position.y]
        print(self.goal_pt)
        self.get_goal = True

    def odom_cb(self, msg):
        self.odom.set(msg.data)

    def run(self):
        rate = rospy.Rate(10) # 10hz

        eps_en = 1
        acc_en = 1
        turn_sig = 0

        state = 'Ready'
        shortest_path = None
        f_shortest_path = None
        g_id, g_idx = None, None

        # velocity plan parameters
        min_v = 7.0 # km/h
        ref_v = 50.0 # km/h

        st_param = {'s_min':-20.0, 's_max':50.0, 't_max':8.0, 'dt':0.2, 'dt_exp':1.0}

        plt.ion()
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlim([0.0, st_param['t_max']])
        ax.set_xticks([i for i in np.arange(0, int(st_param['t_max']), 0.5)])
        ax.grid(color='#BDBDBD', linestyle='-', linewidth=2, )
        drawn = None
        
        test = 0
        init_toggle = True

        while not rospy.is_shutdown():
            # self.odom.fake_pose()
            # print(self.odom.x)
            if state == 'Ready':
                if self.get_goal:
                    rospy.loginfo('Got new goal!')
                    # print(self.tmap.tiles, self.tmap.tile_size, (self.odom.x, self.odom.y))
                    ego_lanelets = lanelet_matching(self.tmap.tiles, self.tmap.tile_size, (self.odom.x, self.odom.y))

                    if ego_lanelets is not None:
                        e_id, e_idx = ego_lanelets
                    else:
                        rospy.loginfo('Failed to match ego to lanelets!')
                        self.get_goal = False
                        continue
                    

                    goal_lanelets = lanelet_matching(self.tmap.tiles, self.tmap.tile_size, self.goal_pt)

                    if goal_lanelets is not None:
                        g_id, g_idx = goal_lanelets
                        goal_viz = GoalViz(self.goal_pt)
                        self.pub_goal.publish(goal_viz)
                    else:
                        rospy.loginfo('Failed to match goal to lanelets!')
                        self.get_goal = False
                        continue

                    e_node = node_matching(self.lmap.lanelets, e_id, e_idx)
                    # print('-------------', e_node)
                    g_node = node_matching(self.lmap.lanelets, g_id, g_idx)

                    if e_node == g_node:
                        shortest_path = ([e_node], 0)
                    else:
                        # modified_graph = modify_graph(self.graph.copy(), self.const_site)
                        shortest_path = dijkstra(self.graph, e_node, g_node)
                        # if init_toggle:
                        #     f_shortest_path = shortest_path
                        # else:
                        #     f_shortest_path += shortest_path
                        # test += 1

                    if shortest_path is not None:
                    # print('test:', test)
                    # if test == 3:
                        shortest_path_viz = ShortestPathViz(self.lmap.lanelets, shortest_path[0])
                        self.pub_shortest_path.publish(shortest_path_viz)

                        state = 'MoveToGoal'
                        rospy.loginfo('Move to goal! Move Cost : %s'%(shortest_path[1]))
                    else:
                        rospy.loginfo('Failed to find shortest path!')

                    self.get_goal = False

            elif state == 'MoveToGoal':
                final_path = []
                for node_id in shortest_path[0]:
                    split_id = node_id.split('_')
                    if len(split_id) == 2:
                        s_idx, e_idx = (self.lmap.lanelets[split_id[0]]['cut_idx'][int(split_id[1])])
                        final_path.append(self.lmap.lanelets[split_id[0]]['waypoints'][int((s_idx+e_idx)//2)])
                        # print('aaaaa',final_path)
                    else:
                        final_path.extend(self.lmap.lanelets[split_id[0]]['waypoints'])
                        rasidual_path = copy.deepcopy(final_path)

                precision = 0.5
                final_path, max_v, last_s = ref_interpolation(final_path, precision, min_v, ref_v)
                ###curvature test
                _, _, yaws, curvatures = interpolate(final_path, precision) 
                ###
                final_path_viz = FinalPathViz(final_path)
                self.pub_final_path.publish(final_path_viz)

                cte, idx = calc_cte_and_idx(final_path, (self.odom.x, self.odom.y))
                s = idx * precision

                # target_v, drawn = velocity_plan(ax, drawn, st_param, ref_v, last_s, s, max_v, self.odom.v, self.odom.a)
             
                # del rasidual_path[:idx+1]

                # if len(rasidual_path) < 15 :
                #     state = "Ready"
                #     # target_v = 0


                ### OVERTAKE
                # a_idx = self.sub_waypoint.query((self.odom.x, self.odom.y),1)[1]

                # speed_variable_in = int(15 * (self.odom.v * 3.6)/50)
                # speed_variable_out = int(25 * (self.odom.v * 3.6)/50)

                # if self.local_state == 0 and self.overtake:
                #     self.local_state = 1

                # if self.local_state == 1:
                #     if len(final_path) - idx > 35:
                #         stitch = self.sub_waypoint.query(final_path[idx+10+speed_variable_in],1)[1]
                #         alpha_path = final_path[:idx + 10 + speed_variable_in] + sub_final_path[stitch + 15 + speed_variable_out:] 
                #         self.local_state = 2
                #         self.overtake = False
                #         self.current_route = 2

                # elif self.local_state == 2 and self.overtake:
                #     if len(sub_final_path) - a_idx > 35:
                #         stitch = self.waypoint.query(sub_final_path[a_idx+10+speed_variable_in],1)[1]
                #         self.local_state = 0
                #         self.overtake = False
                #         self.current_route = 1
                ###

                #print(target_v)

                ##
                ego_lanelets_1 = lanelet_matching(self.tmap.tiles, self.tmap.tile_size, (self.odom.x, self.odom.y))
                print("p_id : " + str(ego_lanelets_1[0] + '_' + str(ego_lanelets_1[1])))
                if ego_lanelets_1 is not None:
                    p_id, p_idx = ego_lanelets_1
                lane_ID = Int16MultiArray()
                lane_ID.data = [p_id]
                # self.pub_current_laneID.publish(lane_ID)
                try:
                    self.pub_curvature.publish(Float32(curvatures[int(p_id)]))
                    print(curvatures[int(p_id)])
                except:
                    pass
                #self.pub_target_v.publish(Float32(target_v))

                fig.canvas.draw()
                lanechange = siginal_light_toggle(final_path, idx, precision, self.tmap, self.lmap, stage=1)
                print('lc : {}'.format(lanechange))


            # can_cmd = Int16MultiArray()
            # can_cmd.data = [eps_en, 0, 190, acc_en, 0, 0, 0, turn_sig]
            # self.pub_can_cmd.publish(can_cmd)

            rate.sleep()


class Arguments:
    def __init__(self):
        self.map_path =''
        self.tile_size = 5.0
        self.cut_dist = 10.0
    def set_map_path(self, name):
            self.map_path = g_path + '/map/%s.json'%(name) 
            
if __name__ == "__main__":
 
    args = Arguments()
    name = 'KIAPI'
    # name = 'songdo'
    # name = 'KCity'
    # name = 'SeoulAutonomousDrivingTestBed'
    # name = 'AlphaCity'
    args.set_map_path(name)
    planner = Planner(args)
    planner.run()
