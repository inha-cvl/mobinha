import tf
from scipy.spatial import KDTree
import time
import rospy
import pymap3d as pm

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseArray, Point
from jsk_recognition_msgs.msg import BoundingBoxArray

from selfdrive.visualize.rviz_utils import *
from selfdrive.perception.libs.obstacle_utils import ObstacleUtils


class ObstacleDetector:
    def __init__(self, CP):

        self.CS = None
        self.CP = CP

        self.local_path = None
        self.lidar_object = []
        self.goal_object = []
        self.traffic_light_object = []

        self.is_morai = False
        self.morai_ego_v = 0.0
        self.traffic_light_timer = time.time()

        self.last_observed_light = []
        self.last_observed_time = 0
        self.frames_of_same_light = 0
        self.allowed_unrecognized_frames = 0
        self.go_signals = [4, 9, 12, 14, 17]
        
        rospy.Subscriber('/mobinha/planning/local_path', Marker, self.local_path_cb)
        # /mobinha/perception
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_box_cb)
        rospy.Subscriber('/mobinha/perception/camera/bounding_box',PoseArray, self.camera_bounding_box_cb)
        rospy.Subscriber('/mobinha/planning/lane_information', Pose, self.lane_information_cb)
        # MORAI
        rospy.Subscriber('/morai/object_list', PoseArray, self.morai_object_list_cb)
        rospy.Subscriber('/morai/ego_topic', Pose, self.morai_ego_topic_cb)
        rospy.Subscriber('/morai/traffic_light', PoseArray,self.morai_traffic_light_cb)
    
        self.pub_object_marker = rospy.Publisher('/mobinha/perception/object_marker', MarkerArray, queue_size=1)
        self.pub_lidar_obstacle = rospy.Publisher('/mobinha/perception/lidar_obstacle', PoseArray, queue_size=1)
        # self.pub_lidar_bsd = rospy.Publisher('/mobinha/perception/lidar_bsd', Point, queue_size=1)
        self.pub_obstacle_distance = rospy.Publisher('/mobinha/perception/nearest_obstacle_distance', Float32, queue_size=1)
        self.pub_traffic_light_obstacle = rospy.Publisher('/mobinha/perception/traffic_light_obstacle', PoseArray, queue_size=1)
        self.pub_around_obstacle = rospy.Publisher('/mobinha/perception/around_obstacle', PoseArray, queue_size=1)

    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def lane_information_cb(self, msg):
        self.lane_change_point = msg.orientation.z

    def lidar_cluster_box_cb(self, msg):
        objects = []
        for obj in msg.boxes:
            x, y = obj.pose.position.x, obj.pose.position.y
            v_rel = obj.value
            track_id = obj.label # uint type 0:clustering 1~: tracking
            w = obj.pose.orientation.z
            sx, sy, sz = obj.dimensions.x, obj.dimensions.y, obj.dimensions.z
            if self.CS is not None:
                nx, ny = ObstacleUtils.object2enu(
                    (self.CS.position.x, self.CS.position.y, self.CS.yawRate), x, y)
                objects.append([nx, ny, w, v_rel, track_id, sx, sy, sz])  # [2] heading [3] relative velocity [4] id

        self.lidar_object = objects

    def camera_bounding_box_cb(self, msg):
        traffic_light_object = []
        for pose in msg.poses:
            cls, size, prob = pose.position.x, pose.position.y, pose.position.z
            if cls != 0:
                traffic_light_object.append([cls, size, prob])
        self.traffic_light_object = traffic_light_object
        self.traffic_light_timer = time.time()

    def morai_object_list_cb(self, msg):
        objects = []
        for obj in msg.poses:
            objects.append((obj.position.x, obj.position.y, 0, (obj.orientation.w/3.6) - self.morai_ego_v, 1, 1,1,1))
        self.lidar_object = objects

    def morai_traffic_light_cb(self, msg):
        traffic_lights = []
        for tl in msg.poses:
            traffic_lights.append(
                (tl.position.x, tl.position.y, tl.position.z))
        self.traffic_light_object = traffic_lights
        self.traffic_light_timer = time.time()

    def morai_ego_topic_cb(self, msg):
        self.is_morai = True
        self.morai_local_point = (msg.position.x, msg.position.y)
        self.morai_ego_v = msg.orientation.w

    def get_lidar_objects(self, local_point, car_idx):
        dx = self.CS.position.x - \
            self.morai_local_point[0] if self.is_morai else 0
        dy = self.CS.position.y - \
            self.morai_local_point[1] if self.is_morai else 0

        viz_obstacle = []
        obstacle_sd = []
        # left_bsd_obstacle_sd = []
        # right_bsd_obstacle_sd = []
        around_obstacle_sd = []
        if len(self.lidar_object) > 0:
            for obj in self.lidar_object:
                obj_s, obj_d = ObstacleUtils.object2frenet(local_point, self.local_path,(obj[0]+dx, obj[1]+dy))
                if obj_d > -4.5 and obj_d < 4.5:
                    #[0] x [1] y [6] s [7] d
                    viz_obstacle.append((obj[0]+dx, obj[1]+dy, obj[2], obj[4], obj[5], obj[6], obj_s-car_idx, obj_d))
                #Forward Collision Warning
                if (obj_s-car_idx) > 0 and (obj_s-car_idx) < 100*(1/self.CP.mapParam.precision) and obj_d > -4.5 and obj_d < 4.5:
                    obstacle_sd.append((obj_s, obj_d, obj[3], obj[4]))
                #BSD3 : Time Based Method
                if (-50*(1/self.CP.mapParam.precision)) <(obj_s-car_idx) < (50*(1/self.CP.mapParam.precision)) and obj_d > -4.5 and obj_d < 4.5:
                        around_obstacle_sd.append((obj_s, obj_d, obj[3]))
                #BSD1 : Ego Position Based Method
                
                # if (-35*(1/self.CP.mapParam.precision)) <(obj_s-car_idx) < (10*(1/self.CP.mapParam.precision)):
                #     #print(obj_s-car_idx, obj_d)
                #     if -4<obj_d<-1:
                #         left_bsd_obstacle_sd.append((obj_s, obj_d, obj[3]))
                #     elif 1<obj_d<4:
                #         right_bsd_obstacle_sd.append((obj_s, obj_d, obj[3]))
                
                #BSD2 : Lange Change Point Based Method
                '''
                if self.lane_change_point-(20*(1/self.CP.mapParam.precision))<obj_s<self.lane_change_point+(5*(1/self.CP.mapParam.precision)):
                    if -4.5<obj_d<-1.5:
                        left_bsd_obstacle_sd.append((obj_s, obj_d, obj[3]))
                    elif 1.5<obj_d<4.5:
                        right_bsd_obstacle_sd.append((obj_s, obj_d, obj[3]))
                '''
                
        # sorting by s
        obstacle_sd = sorted(obstacle_sd, key=lambda sd: sd[0])
        # left_bsd_obstacle_sd = sorted(left_bsd_obstacle_sd, key=lambda sd: sd[0])
        # right_bsd_obstacle_sd = sorted(right_bsd_obstacle_sd, key=lambda sd: sd[0])
        return obstacle_sd, viz_obstacle, around_obstacle_sd

    def process_traffic_lights(self, traffic_light_obs):
        if not traffic_light_obs:
            self.allowed_unrecognized_frames += 1
            if self.allowed_unrecognized_frames <= 2 and self.last_observed_light:
                return [self.last_observed_light]
            elif self.last_observed_light and time.time() - self.last_observed_time < 1.1 and self.last_observed_light[0] in self.go_signals:
                return [self.last_observed_light]
            else:
                return []
        
        current_light = traffic_light_obs[0] if traffic_light_obs else None
        self.allowed_unrecognized_frames = 0

        if current_light is None:
            return []

        if self.last_observed_light and current_light[0] == self.last_observed_light[0]:
            self.frames_of_same_light += 1
        else:
            self.frames_of_same_light = 1

        self.last_observed_light = current_light
        self.last_observed_time = time.time()

        if self.frames_of_same_light >= 3:
            return [current_light]

        if self.last_observed_light[0] in self.go_signals and time.time() - self.last_observed_time < 1.1:
            return [self.last_observed_light]

        return []
    
    def get_traffic_light_objects(self):
        traffic_light_obs = []

        if len(self.traffic_light_object) > 0:
            for traffic_light in self.traffic_light_object:
                if traffic_light[2] > 0.2:  # if probability exceed 20%
                    traffic_light_obs.append(traffic_light)
        # sorting by size
        traffic_light_obs = sorted(traffic_light_obs, key=lambda obs: obs[1], reverse=True)
        
        traffic_light_obs = self.process_traffic_lights(traffic_light_obs)
        return traffic_light_obs

    def run(self, CS):
        self.CS = CS

        if self.local_path is not None:
            local_point = KDTree(self.local_path)
            car_idx = local_point.query(
                (self.CS.position.x, self.CS.position.y), 1)[1]

            obstacle_sd, viz_obstacle, around_obstacle_sd = self.get_lidar_objects(
                local_point, car_idx)

            lidar_obstacle = PoseArray()
            for sd in obstacle_sd:
                pose = Pose()
                pose.position.x = 0  # 0:dynamic
                pose.position.y = sd[0]
                pose.position.z = sd[1]
                pose.orientation.w = sd[2]# relative velocity
                pose.orientation.z = sd[3] # track id
                lidar_obstacle.poses.append(pose)
            # only my front near obstacle distance
            if len(obstacle_sd) > 0:
                if obstacle_sd[0][0]-car_idx < 100*(1/self.CP.mapParam.precision) and -1.6 < obstacle_sd[0][1] < 1.6:
                    obstacle_distance = (obstacle_sd[0][0]-car_idx)*self.CP.mapParam.precision
                else:
                    obstacle_distance = -1
            else:
                obstacle_distance = -1

            # bsd = Point()
            # bsd.x = int(bool(left_bsd_obstacle_sd)) #function converts the list to a boolean value (True if the list is not empty, False if it is)
            # bsd.y = int(bool(right_bsd_obstacle_sd))

            around_obstacle = PoseArray()
            for sd in around_obstacle_sd:
                pose = Pose()
                pose.position.x = 0  # 0:dynamic
                pose.position.y = sd[0]
                pose.position.z = sd[1]
                pose.orientation.w = sd[2]# relative velocity
                around_obstacle.poses.append(pose)
                
            # TODO: Traffic Light
            traffic_light_obs = self.get_traffic_light_objects()
            traffic_light_obstacle = PoseArray()
            for tl in traffic_light_obs:
                pose = Pose()
                pose.position.x = 2  # traffic light
                pose.position.y = tl[0]  # cls
                pose.position.z = tl[2]  # probability
                traffic_light_obstacle.poses.append(pose)

            self.pub_obstacle_distance.publish(Float32(obstacle_distance))
            self.pub_lidar_obstacle.publish(lidar_obstacle)
            # self.pub_lidar_bsd.publish(bsd)
            self.pub_around_obstacle.publish(around_obstacle)
            objects_viz = ObjectsViz(viz_obstacle)
            self.pub_object_marker.publish(objects_viz)
            self.pub_traffic_light_obstacle.publish(traffic_light_obstacle)
            

            # self.lidar_object = []
            viz_obstacle = []
            if time.time()-self.traffic_light_timer > 3:
                self.traffic_light_object = []


