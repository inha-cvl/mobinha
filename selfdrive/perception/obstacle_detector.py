from scipy.spatial import KDTree
import math
import rospy
from geometry_msgs.msg import Pose, PoseArray
from jsk_recognition_msgs.msg import BoundingBoxArray

from selfdrive.visualize.viz import *

class ObstacleDetector:
    def __init__(self):

        self.CS = None

        self.local_path = None
        self.lidar_object = []
        self.lidar_obstacle = PoseArray()

        self.sub_local_path = rospy.Subscriber('/local_path', Marker, self.local_path_cb)
        self.sub_lidar_cluster_box = rospy.Subscriber('/lidar/cluster_box', BoundingBoxArray, self.lidar_cluster_box_cb)
        self.pub_object_marker = rospy.Publisher('/object_marker', MarkerArray, queue_size=2)

        self.pub_lidar_obstacle = rospy.Publisher('/lidar_obstacle', PoseArray, queue_size =1)
    
    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def lidar_cluster_box_cb(self, msg):
        objects = []
        for _, obj in enumerate(msg.boxes):
            x, y = obj.pose.position.x, obj.pose.position.y
            if self.CS is not None:
                nx, ny = self.object2enu((self.CS.position.x, self.CS.position.y, self.CS.yawRate), x, y)
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
    
    def run(self, sm):
        self.CS = sm.CS

        if self.local_path is not None:
            local_point = KDTree(self.local_path)

            zip_obstacle = []
            if len(self.lidar_object) > 0:
                for _, obj in enumerate(self.lidar_object):
                    obj_s, obj_d = self.object2frenet(
                        local_point, self.local_path, obj[0], obj[1])
                    if obj_s < 30 and obj_d > -1.5 and obj_d < 1.5:
                        pose = Pose()
                        pose.position.x = 0
                        pose.position.y = obj_s
                        pose.position.z = obj_d
                        self.lidar_obstacle.append(pose)
                        zip_obstacle.append(zip(_, (obj[:1])))
            
            self.pub_lidar_obstacle.publish(self.lidar_obstacle)
            objects_viz = ObjectsViz(zip_obstacle)
            self.pub_object_marker.publish(objects_viz)
