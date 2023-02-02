from scipy.spatial import KDTree
import math
import tf
import rospy
import pymap3d as pm

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseArray
from jsk_recognition_msgs.msg import BoundingBoxArray

from selfdrive.visualize.viz_utils import *
from selfdrive.perception.libs.obstacle_utils import ObstacleUtils

IDX_TO_M = 0.5
M_TO_IDX = 2


class ObstacleDetector:
    def __init__(self, CP):

        self.CS = None
        self.CP = CP

        self.local_path = None
        self.lidar_object = []
        self.goal_object = []

        self.is_morai = False

        rospy.Subscriber(
            '/mobinha/planning/local_path', Marker, self.local_path_cb)
        rospy.Subscriber(
            '/mobinha/perception/lidar/cluster_box', BoundingBoxArray, self.lidar_cluster_box_cb)

        #MORAI
        rospy.Subscriber(
            '/morai/object_list', PoseArray, self.morai_object_list_cb)
        rospy.Subscriber(
            '/morai/ego_topic', Pose, self.morai_ego_topic_cb)

        self.pub_object_marker = rospy.Publisher(
            '/mobinha/perception/object_marker', MarkerArray, queue_size=1)

        self.pub_lidar_obstacle = rospy.Publisher(
            '/mobinha/perception/lidar_obstacle', PoseArray, queue_size=1)
        self.pub_obstacle_distance = rospy.Publisher(
            '/mobinha/perception/nearest_obstacle_distance', Float32, queue_size=1)

    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def lidar_cluster_box_cb(self, msg):
        objects = []
        for obj in msg.boxes:
            x, y = obj.pose.position.x, obj.pose.position.y
            if self.CS is not None:
                nx, ny = ObstacleUtils.object2enu(
                    (self.CS.position.x, self.CS.position.y, self.CS.yawRate), x, y)
                objects.append([nx, ny, 60])

        self.lidar_object = objects



    def morai_object_list_cb(self, msg):
        objects = []
        for obj in msg.poses:
            objects.append((obj.position.x, obj.position.y, obj.position.z))
        self.lidar_object = objects

    def morai_ego_topic_cb(self, msg):
        self.is_morai = True
        self.morai_local_point = (msg.position.x, msg.position.y)

    def run(self, CS):
        self.CS = CS

        if self.local_path is not None:
            local_point = KDTree(self.local_path)

            car_idx = local_point.query(
                (self.CS.position.x, self.CS.position.y), 1)[1]

            dx = self.CS.position.x - \
                self.morai_local_point[0] if self.is_morai else 0
            dy = self.CS.position.y - \
                self.morai_local_point[1] if self.is_morai else 0

            viz_obstacle = []
            obstacle_sd = []
            if len(self.lidar_object) > 0:
                for obj in self.lidar_object:
                    obj_s, obj_d = ObstacleUtils.object2frenet(
                        local_point, self.local_path, (obj[0]+dx, obj[1]+dy))
                    if (obj_s-car_idx) > 0 and (obj_s-car_idx) < 100*M_TO_IDX and obj_d > -3.5 and obj_d < 3.5:
                        obstacle_sd.append((obj_s, obj_d))
                        viz_obstacle.append((obj[0]+dx, obj[1]+dy, obj[2]))

            # TODO: Traffic Light
            sorted_obstacle_sd = sorted(obstacle_sd, key=lambda sd: sd[0])

            lidar_obstacle = PoseArray()
            for sd in sorted_obstacle_sd:
                pose = Pose()
                pose.position.x = 0  # 0:dynamic, 1:static
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
