import copy

import rospy
from visualization_msgs.msg import MarkerArray

from libs.planner_utils import *
from selfdrive.visualize.rviz_utils import *


class MicroLaneletGraph:
    def __init__(self, lmap, cut_dist):
        self.cut_dist = cut_dist
        self.precision = lmap.precision
        self.lanelets = lmap.lanelets
        self.groups = lmap.groups
        self.generate_micro_lanelet_graph()

        self.pub_micro_lanelet_graph = rospy.Publisher(
            '/micro_lanelet_graph', MarkerArray, queue_size=1, latch=True)
        micro_lanelet_graph_viz = MicroLaneletGraphViz(
            self.lanelets, self.graph)
        self.pub_micro_lanelet_graph.publish(micro_lanelet_graph_viz)

    def generate_micro_lanelet_graph(self):
        self.graph = {}
        cut_idx = int(self.cut_dist / self.precision)

        for group in self.groups:
            group = copy.copy(group)

            if self.lanelets[group[0]]['length'] > self.lanelets[group[-1]]['length']:
                group.reverse()

            idx_num = self.lanelets[group[0]]['idx_num']

            cut_num = idx_num // cut_idx
            if idx_num % cut_idx != 0:
                cut_num += 1

            for n, id_ in enumerate(group):
                self.lanelets[id_]['cut_idx'] = []

                if n == 0:
                    for i in range(cut_num):
                        start_idx = i * cut_idx

                        if i == cut_num - 1:
                            end_idx = idx_num
                        else:
                            end_idx = start_idx + cut_idx

                        self.lanelets[id_]['cut_idx'].append(
                            [start_idx, end_idx])

                else:
                    for i in range(cut_num):
                        pre_id = group[n-1]
                        pre_end_idx = self.lanelets[pre_id]['cut_idx'][i][1] - 1

                        pt = self.lanelets[pre_id]['waypoints'][pre_end_idx]

                        if i == 0:
                            start_idx = 0
                            end_idx = find_nearest_idx(
                                self.lanelets[id_]['waypoints'], pt)

                        elif i == cut_num - 1:
                            start_idx = self.lanelets[id_]['cut_idx'][i-1][1]
                            end_idx = self.lanelets[id_]['idx_num']

                        else:
                            start_idx = self.lanelets[id_]['cut_idx'][i-1][1]
                            end_idx = find_nearest_idx(
                                self.lanelets[id_]['waypoints'], pt)

                        self.lanelets[id_]['cut_idx'].append(
                            [start_idx, end_idx])

        for id_, data in self.lanelets.items():
            if data['group'] is None:
                if self.graph.get(id_) is None:
                    self.graph[id_] = {}

                for p_id in data['successor']:
                    if self.lanelets[p_id]['group'] is None:
                        self.graph[id_][p_id] = data['length']
                    else:
                        self.graph[id_][p_id+'_0'] = data['length']

            else:
                last = len(data['cut_idx']) - 1
                for n in range(len(data['cut_idx'])):
                    new_id = '%s_%s' % (id_, n)
                    if self.graph.get(new_id) is None:
                        self.graph[new_id] = {}

                    if n == last:
                        for p_id in data['successor']:
                            if self.lanelets[p_id]['group'] is None:
                                self.graph[new_id][p_id] = self.cut_dist
                            else:
                                self.graph[new_id][p_id+'_0'] = self.cut_dist

                    else:
                        self.graph[new_id]['%s_%s' %
                                           (id_, n+1)] = self.cut_dist

                        s_idx, e_idx = self.lanelets[id_]['cut_idx'][n]

                        left_id = data['adjacentLeft']
                        if left_id is not None:
                            if sum(self.lanelets[id_]['leftChange'][s_idx:s_idx+(e_idx-s_idx)//2]) == (e_idx - s_idx)//2:
                                self.graph[new_id]['%s_%s' % (
                                    left_id, n+1)] = self.cut_dist + 10.0 + n * 0.1

                        right_id = data['adjacentRight']
                        if right_id is not None:
                            if sum(self.lanelets[id_]['rightChange'][s_idx:s_idx+(e_idx-s_idx)//2]) == (e_idx - s_idx)//2:
                                self.graph[new_id]['%s_%s' % (
                                    right_id, n+1)] = self.cut_dist + 10.0 + n * 0.1

        self.reversed_graph = {}
        for from_id, data in self.graph.items():
            for to_id in data:
                if self.reversed_graph.get(to_id) is None:
                    self.reversed_graph[to_id] = []

                self.reversed_graph[to_id].append(from_id)
