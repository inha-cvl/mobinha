import rospy
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

        self.ref_v = CP.maxEnableSpeed
        self.min_v = CP.minEnableSpeed
        self.target_v = self.min_v*KPH_TO_MPS
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

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z)
                               for pose in msg.poses]

    def traffic_light_obstacle_cb(self, msg):
        self.traffic_light_obstacle = [
            (pose.position.x, pose.position.y, pose.position.z) for pose in msg.poses]

    def lane_information_cb(self, msg):
        # [0] id, [1] forward_direction, [2] cross_walk distance [3] forward_curvature
        self.lane_information = [msg.position.x,
                                 msg.position.y, msg.position.z, msg.orientation.x]

    def goal_object_cb(self, msg):
        self.goal_object = (msg.position.x, msg.position.y, msg.position.z)

    def obstacle_handler(self, obj, s, cur_v):
        # [0] Dynamic [1] Static [2] Traffic Light
        i = int(obj[0])
        offset = [15, 5, 10]  # m
        offset = [os*self.M_TO_IDX for os in offset]
        pos = obj[1] + s if obj[0] == 1 else obj[1]
        return pos-offset[i]

    def sigmoid_logit_function(self, s):
        return ((1+((s*self.sl_param["mu"])/(self.sl_param["mu"]*(1-s)))**-self.sl_param["v"])**-1)

    def simple_velocity_plan(self, cur_v, max_v,  local_s, object_list):
        pi = 1
        min_obs_s = 1
        consider_distance = 70*self.M_TO_IDX
        for obj in object_list:
            s = self.obstacle_handler(
                obj, local_s, cur_v) - local_s  # Remain Distance
            # print(s)
            norm_s = 1
            if 0 < s < consider_distance:
                norm_s = s/consider_distance
            elif s <= 0:
                norm_s = 0
            if min_obs_s > norm_s:
                min_obs_s = norm_s
        # print(min_obs_s)
        if 0 < min_obs_s < 1:
            pi = self.sigmoid_logit_function(min_obs_s)
        elif min_obs_s <= 0:
            pi = 0

        target_v = max_v * pi

        gain = 0.3
        if self.target_v-target_v < -gain:
            target_v = self.target_v + gain
        elif self.target_v-target_v > gain:
            target_v = self.target_v - gain

        if target_v > self.ref_v*KPH_TO_MPS:
            target_v = self.ref_v*KPH_TO_MPS
        elif target_v < 0.3:
            target_v = 0

        return target_v

    def traffic_light_to_obstacle(self, traffic_light, forward_direction):
        # TODO: consideration filtering
        consideration_class_list = [[6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13], [
            6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13]]
        if traffic_light in consideration_class_list[forward_direction]:
            return True
        else:
            return False

    def check_objects(self, local_len):
        object_list = []

        # [0] = Dynamic Object
        if self.lidar_obstacle is not None:
            for lobs in self.lidar_obstacle:
                if lobs[2] >= -2.0 and lobs[2] <= 2.0:  # object in my lane
                    object_list.append(lobs)

        # [1] = Goal Object
        if self.goal_object is not None:
            left = (self.goal_object[1]-self.goal_object[2]) * self.M_TO_IDX
            if left <= local_len:
                object_list.append(
                    [self.goal_object[0], left, 0])

        # [2] = Traffic Light
        if self.traffic_light_obstacle is not None:
            for tlobs in self.traffic_light_obstacle:
                if self.traffic_light_to_obstacle(int(tlobs[1]), int(self.lane_information[1])):
                    object_list.append((tlobs[0], self.lane_information[2], 0))
        return object_list

    def run(self, sm, pp=0, local_path=None):
        CS = sm.CS
        lgp = 0
        self.pub_target_v.publish(Float32(self.target_v))

        if local_path != None and self.lane_information != None:
            local_idx = calc_idx(
                local_path, (CS.position.x, CS.position.y))
            local_curv_v = max_v_by_curvature(
                self.lane_information[3], self.ref_v, self.min_v)
            object_list = self.check_objects(len(local_path))

            self.target_v = self.simple_velocity_plan(
                CS.vEgo, self.ref_v*KPH_TO_MPS, local_idx, object_list)

            if self.target_v > local_curv_v:
                self.target_v = local_curv_v

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.0001:
                    lgp = 2
            elif pp == 4:
                self.target_v = 0.0
            else:
                lgp = 1

        return lgp
