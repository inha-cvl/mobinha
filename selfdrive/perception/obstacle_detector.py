from scipy.spatial import KDTree
import math
import rospy
import pymap3d as pm

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseArray
from jsk_recognition_msgs.msg import BoundingBoxArray

from selfdrive.visualize.viz_utils import *

IDX_TO_M = 0.5
M_TO_IDX = 2


class ObstacleDetector:
    def __init__(self, CP):

        self.CS = None
        self.CP = CP

        self.local_path = None
        self.lidar_object = []

        self.sub_local_path = rospy.Subscriber(
            '/local_path', Marker, self.local_path_cb)
        self.sub_lidar_cluster_box = rospy.Subscriber(
            '/lidar/cluster_box', BoundingBoxArray, self.lidar_cluster_box_cb)
        self.pub_object_marker = rospy.Publisher(
            '/object_marker', MarkerArray, queue_size=2)

        self.pub_lidar_obstacle = rospy.Publisher(
            '/lidar_obstacle', PoseArray, queue_size=1)
        self.pub_obstacle_distance = rospy.Publisher(
            '/nearest_obstacle_distance', Float32, queue_size=1)

    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def lidar_cluster_box_cb(self, msg):
        objects = []
        for obj in msg.boxes:
            x, y = obj.pose.position.x, obj.pose.position.y
            if self.CS is not None:
                nx, ny = self.object2enu(
                    (self.CS.position.x, self.CS.position.y, self.CS.yawRate), x, y)
                objects.append([nx, ny])

        self.lidar_object = objects

    def object2enu(self, odom, obj_local_y, obj_local_x):
        rad = odom[2] * (math.pi / 180.0)

        nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
        ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

        obj_x = odom[0] + ny
        obj_y = odom[1] + nx

        return obj_x, obj_y

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def object2frenet(self, local_point, local_path, obj_x, obj_y):  # Array in Frenet Array Out
        target = [obj_x, obj_y]
        l_idx = local_point.query(target, 1)[1]
        if(l_idx == 0):
            return 0, 1000
        wp = local_path

        n_x = wp[l_idx][0] - wp[l_idx-1][0]
        n_y = wp[l_idx][1] - wp[l_idx-1][1]
        x_x = target[0] - wp[l_idx-1][0]
        x_y = target[1] - wp[l_idx-1][1]

        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y

        frenet_d = self.distance(x_x, x_y, proj_x, proj_y)

        center_x = 900 - wp[l_idx-1][0]
        center_y = -100 - wp[l_idx-1][1]
        distToPos = self.distance(center_x, center_y, x_x, x_y)
        distToRef = self.distance(center_x, center_y, proj_x, proj_y)

        if(distToPos > distToRef):
            frenet_d *= -1

        return l_idx, frenet_d

    def run(self, CS):
        self.CS = CS

        if self.local_path is not None:
            local_point = KDTree(self.local_path)
            car_idx = local_point.query(
                (self.CS.position.x, self.CS.position.y), 1)[1]

            viz_obstacle = []
            obstacle_sd = []
            if len(self.lidar_object) > 0:
                for obj in self.lidar_object:
                    obj_s, obj_d = self.object2frenet(
                        local_point, self.local_path, obj[0], obj[1])

                    if (obj_s-car_idx) < 60*M_TO_IDX and obj_d > -2.5 and obj_d < 2.5:
                        obstacle_sd.append((obj_s, obj_d))
                        viz_obstacle.append((obj[:2]))

            sorted_obstacle_sd = sorted(obstacle_sd, key=lambda sd: sd[0])

            lidar_obstacle = PoseArray()
            for sd in sorted_obstacle_sd:
                pose = Pose()
                pose.position.x = 0
                pose.position.y = sd[0]
                pose.position.z = sd[1]
                lidar_obstacle.poses.append(pose)

            if len(lidar_obstacle.poses) > 0:
                obstacle_distance = (
                    lidar_obstacle.poses[0].position.y-car_idx)*IDX_TO_M
            else:
                obstacle_distance = -1
            self.pub_obstacle_distance.publish(Float32(obstacle_distance))
            self.pub_lidar_obstacle.publish(lidar_obstacle)
            objects_viz = ObjectsViz(viz_obstacle)
            self.pub_object_marker.publish(objects_viz)