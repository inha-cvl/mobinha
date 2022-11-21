import numpy as np
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose, PoseArray

from libs.map import LaneletMap
from libs.planner_utils import *
from selfdrive.visualize.viz import *

class LongitudinalPlanner:
    def __init__(self, CP):
        self.lmap = LaneletMap(CP.mapParam.path)

        self.local_path = None
        self.lidar_obstacle = None
        self.now_lane_id = ''

        self.min_v = CP.minEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.target_v = 0.0
        self.st_param = CP.stParam._asdict()
        self.precision = CP.mapParam.precision
        
        self.sub_local_path = rospy.Subscriber(
            '/local_path', Marker, self.local_path_cb)
        self.sub_now_lane_id = rospy.Subscriber(
            '/now_lane_id', String, self.now_lane_id_cb
        )
        self.sub_lidar_obstacle = rospy.Subscriber('/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        
        self.pub_target_v = rospy.Publisher(
            '/target_v', Float32, queue_size=1, latch=True)

    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]
    
    def now_lane_id_cb(self, msg):
        self.now_lane_id = msg.data

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.x, pose.y, pose.z) for pose in msg.poses.position]

    def object2enu(self, odom, obj_local_y, obj_local_x):
        rad = odom["yaw"] * (math.pi / 180.0)

        nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
        ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

        obj_x = odom["x"] + ny
        obj_y = odom["y"] + nx

        return obj_x, obj_y

    def run(self, sm, pp=0):
        CS = sm.CS
        lgp = 0

        self.pub_target_v.publish(Float32(self.target_v))

        if self.local_path is None:
            rospy.loginfo("MOVE")
            _, l_idx = calc_cte_and_idx(
                self.local_path, (CS.position.x, CS.position.y))
            local_s = l_idx+2

            split_now_lane_id = self.now_lane_id.split('_')[0]
            speed_limit = (
                self.lmap.lanelets[split_now_lane_id]['speedLimit'])
            local_max_v = ref_to_max_v(
                self.local_path, self.precision, 1, self.min_v, self.ref_v, speed_limit)

            tl_objects = [15, 30,2] #s, time, state

            #self.target_v = new_velocity_plan(self.ref_v, CD.localLasts, local_s, tl_s, lidar_obstacle)
            self.target_v = velocity_plan(
                self.st_param, self.ref_v, len(self.local_path), local_s, local_max_v, CS.vEgo, CS.aEgo, tl_objects, self.lidar_obstacle)

            if pp == 2 and CS.vEgo <= 1.0:
                self.target_v = 0.0
                if CS.vEgo <= 0.001:
                    lgp = 2
            else:
                lgp = 1

        return lgp
