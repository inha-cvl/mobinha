import math

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray


from selfdrive.message.car_message import car_driving

CD = car_driving.CarDriving()


class DrivingMaster:
    def __init__(self):
        self.CD = CD
        self.CS = None
        self.car_driving = CD._asdict()

        self.local_path = []
        self.local_last_s = len(self.local_path)-1
        self.now_lane_id = 0
        self.lidar_object = []

        self.path_planning_state = 0
        self.longitudinal_planning_state = 0

        rospy.Subscriber('/lidar/cluster_box', BoundingBoxArray,
                         self.lidar_cluster_box_cb)

    def object2enu(self, odom, obj_local_y, obj_local_x):
        rad = odom["yaw"] * (math.pi / 180.0)

        nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
        ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

        obj_x = odom["x"] + ny
        obj_y = odom["y"] + nx

        return obj_x, obj_y

    def lidar_cluster_box_cb(self, msg):
        objects = []
        for _, obj in enumerate(msg.boxes):
            x, y = obj.pose.position.x, obj.pose.position.y
            if self.CS is not None:
                nx, ny = self.object2enu(
                    (self.CS.position.x, self.CS.position.y, self.CS.yawRate), x, y)
                objects.append([nx, ny])
        self.lidar_object = objects

    def update(self, sm=None):
        if sm is not None:
            self.CS = sm.CS
        self.car_driving["localPath"] = self.local_path
        self.car_driving["localLasts"] = self.local_last_s
        self.car_driving["nowLaneID"] = self.now_lane_id
        self.car_driving["lidarObjects"] = self.lidar_object
        self.car_driving["pathPlanningState"] = self.path_planning_state
        self.car_driving["longitudinalPlanningState"] = self.longitudinal_planning_state
        self.CD = self.CD._make(self.car_driving.values())

    def setter(self, data):
        self.local_path = data["localPath"]
        self.local_last_s = data["localLasts"]
        self.now_lane_id = data["nowLaneID"]
        self.path_planning_state = data["pathPlanningState"]
        self.longitudinal_planning_state = data["longitudinalPlanningState"]
        self.update()
