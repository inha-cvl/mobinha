import rospy
import math
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, Pose

from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.viz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


class LongitudinalPlanner:
    def __init__(self, CP):

        self.lidar_obstacle = None
        self.traffic_light_obstacle = None
        self.lane_information = None
        self.goal_object = None
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision
        self.long_timer = 0
        self.long_dt = 0

        self.ref_v = CP.maxEnableSpeed
        self.min_v = CP.minEnableSpeed
        self.target_v = 0  # self.min_v*KPH_TO_MPS
        self.st_param = CP.stParam._asdict()
        self.sl_param = CP.slParam._asdict()

        rospy.Subscriber(
            '/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber('/mobinha/perception/traffic_light_obstacle',
                         PoseArray, self.traffic_light_obstacle_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',
                         Pose, self.lane_information_cb)
        rospy.Subscriber(
            '/mobinha/planning/goal_information', Pose, self.goal_object_cb)
        self.pub_target_v = rospy.Publisher(
            '/mobinha/planning/target_v', Float32, queue_size=1, latch=True)
        self.pub_traffic_light_marker = rospy.Publisher(
            '/mobinha/planner/traffic_light_marker', Marker, queue_size=1)

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z)
                               for pose in msg.poses]

    def traffic_light_obstacle_cb(self, msg):
        self.traffic_light_obstacle = [(pose.position.x, pose.position.y, pose.position.z)
                                       for pose in msg.poses]

    def lane_information_cb(self, msg):
        # [0] id, [1] forward_direction, [2] cross_walk distance [3] forward_curvature
        self.lane_information = [msg.position.x,
                                 msg.position.y, msg.position.z, msg.orientation.x]

    def goal_object_cb(self, msg):
        self.goal_object = (msg.position.x, msg.position.y, msg.position.z)

    def obstacle_handler(self, obj, s, cur_v):
        # [0] Dynamic [1] Static [2] Traffic Light
        i = int(obj[0])
        offset = [8+(1.8*cur_v), 5, 10]  # m
        offset = [os*self.M_TO_IDX for os in offset]
        pos = obj[1] + s if obj[0] == 1 else obj[1]
        return i, pos-offset[i]

    def sigmoid_logit_function(self, s):
        return ((1+((s*(1-self.sl_param["mu"]))/(self.sl_param["mu"]*(1-s)))**-self.sl_param["v"])**-1)

    def dynamic_consider_range(self, max_v, base_range=40):  # input max_v unit (m/s)
        return base_range*self.M_TO_IDX + (0.267*(max_v)**1.902)*self.M_TO_IDX

    def simple_velocity_plan(self, cur_v, max_v,  local_s, object_list):
        pi = 1
        min_obs_s = 1
        consider_distance = self.dynamic_consider_range(self.ref_v*KPH_TO_MPS)
        obj_i = -1
        for obj in object_list:
            obj_i, s = self.obstacle_handler(
                obj, local_s, cur_v)  # Remain Distance
            s -= local_s
            norm_s = 1
            if 0 < s < consider_distance:
                norm_s = s/consider_distance
            elif s <= 0:
                norm_s = 0
            if min_obs_s > norm_s:
                min_obs_s = norm_s
        if 0 < min_obs_s < 1:
            pi = self.sigmoid_logit_function(min_obs_s)
        elif min_obs_s <= 0:
            pi = 0

        target_v = max_v * pi

        # gain = 0.15
        # if obj_i == 0:
        #     gain = 0.3
        # elif obj_i == 2:
        #     gain = 0.2

        # ignore_gain = gain + 0.05
        # if self.target_v-target_v < -gain:
        #     if self.target_v > ignore_gain:
        #         target_v = self.target_v + gain
        #     else:
        #         target_v = self.target_v + ignore_gain
        # elif self.target_v-target_v > gain:
        #     target_v = self.target_v - gain

        # if target_v > self.ref_v*KPH_TO_MPS:
        #     target_v = self.ref_v*KPH_TO_MPS
        # elif target_v < ignore_gain:
        #     target_v = 0

        return target_v

    def dynamic_velocity_planner(self, cur_v, max_v, local_s, object_list):
        obs_s = 80*self.M_TO_IDX
        for obj in object_list:
            obj_i, s = self.obstacle_handler(
                obj, local_s, cur_v)  # Remain Distance
            s -= local_s
            if s < obs_s:
                obs_s = s

        ttc = min(20, (obs_s*self.IDX_TO_M)/cur_v)

        if self.long_timer != 0:
            self.long_dt += time.time()-self.long_timer
        elif self.long_dt < ttc:
            self.long_dt += 0.1

        pi = self.sigmoid_logit_function(self.long_dt/ttc)





    def traffic_light_to_obstacle(self, traffic_light, forward_direction):
        # TODO: consideration filtering
        consideration_class_list = [[6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13], [
            6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13]]
        if traffic_light in consideration_class_list[forward_direction]:
            return False
        else:
            return True

    def check_objects(self, local_path):
        object_list = []
        local_len = len(local_path)
        # [0] = Dynamic Object
        if self.lidar_obstacle is not None:
            for lobs in self.lidar_obstacle:
                if lobs[2] >= -1.5 and lobs[2] <= 1.5:  # object in my lane
                    object_list.append(lobs)

        # [1] = Goal Object
        if self.goal_object is not None:
            left = (self.goal_object[1]-self.goal_object[2]) * self.M_TO_IDX
            if left <= local_len:
                object_list.append([1, left, 0])

        # [2] = Traffic Light
        if self.traffic_light_obstacle is not None:
            can_go = False
            if len(self.traffic_light_obstacle) > 0:
                tlobs = self.traffic_light_obstacle[0]
                if self.traffic_light_to_obstacle(int(tlobs[1]), int(self.lane_information[1])):
                    can_go = True
            if not can_go:
                if self.lane_information[2] <= math.inf:
                    object_list.append((2, self.lane_information[2], 0))
        return object_list

    def run(self, sm, pp=0, local_path=None):
        CS = sm.CS
        lgp = 0
        self.pub_target_v.publish(Float32(self.target_v))
        if local_path != None and self.lane_information != None and CS.cruiseState == 1:
            local_idx = calc_idx(
                local_path, (CS.position.x, CS.position.y))
            local_curv_v = max_v_by_curvature(
                self.lane_information[3], self.ref_v, self.min_v)
            object_list = self.check_objects(local_path)

            self.target_v = self.simple_velocity_plan(
                CS.vEgo,  local_curv_v, local_idx, object_list)

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.001:
                    lgp = 2
            elif pp == 4:
                self.target_v = 0.0
            else:
                lgp = 1
            self.long_timer = time.time()
        return lgp
