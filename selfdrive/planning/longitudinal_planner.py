from scipy.spatial import KDTree
import numpy as np
import rospy
from std_msgs.msg import Float32
from libs.map import LaneletMap
from selfdrive.visualize.viz import *
from libs.planner_utils import *


class LongitudinalPlanner:
    def __init__(self, CP):
        self.lmap = LaneletMap(CP.mapParam.path)

        self.min_v = CP.minEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.target_v = 0.0
        self.st_param = CP.stParam._asdict()
        self.precision = CP.mapParam.precision

        self.pub_object_marker = rospy.Publisher(
            '/object_marker', MarkerArray, queue_size=2)
        self.pub_target_v = rospy.Publisher(
            '/target_v', Float32, queue_size=1, latch=True)

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def object2frenet(self, local_point, local_path, obj_x, obj_y):  # Array in Frenet Array Out
        target = [obj_x, obj_y]
        point = local_point.query(target, 1)[1]
        if(point == 0):
            return 0, 1000
        wp = local_path

        n_x = wp[point][0] - wp[point-1][0]
        n_y = wp[point][1] - wp[point-1][1]
        x_x = target[0] - wp[point-1][0]
        x_y = target[1] - wp[point-1][1]

        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y

        frenet_d = self.distance(x_x, x_y, proj_x, proj_y)

        center_x = 900 - wp[point-1][0]
        center_y = -100 - wp[point-1][1]
        distToPos = self.distance(center_x, center_y, x_x, x_y)
        distToRef = self.distance(center_x, center_y, proj_x, proj_y)

        if(distToPos > distToRef):
            frenet_d *= -1

        return int(point/2), frenet_d

    def object2enu(self, odom, obj_local_y, obj_local_x):
        rad = odom["yaw"] * (math.pi / 180.0)

        nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
        ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

        obj_x = odom["x"] + ny
        obj_y = odom["y"] + nx

        return obj_x, obj_y

    def run(self, sm, dm):
        CS = sm.CS
        CD = dm.CD

        car_driving = CD._asdict()
        self.pub_target_v.publish(Float32(self.target_v))

        if len(CD.localPath) > 0:
            rospy.loginfo("MOVE")
            _, l_idx = calc_cte_and_idx(
                CD.localPath, (CS.position.x, CS.position.y))
            local_s = l_idx+2
            local_point = KDTree(CD.localPath)

            lidar_obstacle = []
            zip_obstacle = []
            if len(CD.lidarObjects) > 0:
                for _, obj in enumerate(CD.lidarObjects):
                    obj_s, obj_d = self.object2frenet(
                        local_point, CD.localPath, obj[0], obj[1])
                    if obj_s < 30 and obj_d > -1.5 and obj_d < 1.5:
                        lidar_obstacle.append([0, obj_s, obj_d])
                        zip_obstacle.append(zip(_, (obj[:1])))

            objects_viz = ObjectsViz(zip_obstacle)
            self.pub_object_marker.publish(objects_viz)

            speed_limit = (
                self.lmap.lanelets[CD.nowLaneID]['speedLimit'])
            local_max_v = ref_to_max_v(
                CD.localPath, self.precision, 1, self.min_v, self.ref_v, speed_limit)

            for i in range(len(local_max_v)):
                local_max_v[i]

            tl_s = 15
            tl_time = 30
            tl_state = 2
            tl_objects = [tl_s, tl_time, tl_state]

            #self.target_v = new_velocity_plan(self.ref_v, CD.localLasts, local_s, tl_s, lidar_obstacle)
            self.target_v = velocity_plan(
                self.st_param, self.ref_v, CD.localLasts, local_s, local_max_v, CS.vEgo, CS.aEgo, tl_objects, lidar_obstacle)

            car_driving["longitudinalPlanningState"] = 1
            dm.setter(car_driving)

            if CD.pathPlanningState == 2 and CS.vEgo <= 1.0:
                self.target_v = 0.0
                if CS.vEgo <= 0.001:
                    car_driving["longitudinalPlanningState"] = 2
                    dm.setter(car_driving)
