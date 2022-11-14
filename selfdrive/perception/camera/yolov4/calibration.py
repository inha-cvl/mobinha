import cv2
import numpy as np
from sklearn import linear_model


class Plane3D:
    def __init__(self, data):
        XY = data[:,:2]
        Z  = data[:,2]
        self.ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(),residual_threshold=0.1)
        self.ransac.fit(XY, Z)

    def get_z(self, x, y):
        return self.ransac.predict(np.array([[x, y]]))


class Calibration:
    def __init__(self, path1, path2):
        # camera parameters
        cam_param = []
        with open(path1, 'r') as f:
            for i in f.readlines():
                for val in i.split(','):
                    cam_param.append(float(val))

        self.camera_matrix = np.array([[cam_param[0], cam_param[1], cam_param[2]], 
                                       [cam_param[3], cam_param[4], cam_param[5]], 
                                       [cam_param[6], cam_param[7], cam_param[8]]])
        self.dist_coeffs = np.array([[cam_param[9]], [cam_param[10]], [cam_param[11]], [cam_param[12]]])

        # calibration parameters
        calib_param = []
        with open(path2, 'r') as f:
            for line in f.readlines():
                calib_param.extend([float(i) for i in line.split(',')])

        RPT = np.array([[calib_param[0], calib_param[1], calib_param[2], calib_param[9]],
                        [calib_param[3], calib_param[4], calib_param[5], calib_param[10]],
                        [calib_param[6], calib_param[7], calib_param[8], calib_param[11]]])

        self.proj_lidar2cam = np.dot(self.camera_matrix, RPT)

        # ground estimation using RANSAC
        ground = np.array([[15.401, -0.82035, -1.5316], 
                           [15.226, 1.8142, -1.5227],
                           [5.2479, 1.674, -1.6736],
                           [15.294, 1.5374, -1.5264],
                           [10.793, -1.553, -1.5937],
                           [5.5749, -3.3064, -1.7005],
                           [8.8118, -0.44693, -1.6224],
                           [6.1645, -3.977, -1.698]])
        self.plane = Plane3D(ground)

        # topview parameters
        self.pt1 = [15.0, 4.0]  # left top
        self.pt2 = [15.0, -4.0] # right top
        self.pt3 = [4.5, -4.0]  # right bottom 
        self.pt4 = [4.5, 4.0]   # left bottom

        for i in [self.pt1, self.pt2, self.pt3, self.pt4]:
            i.append(self.plane.get_z(i[0], i[1]))

        pts_lidar = np.array([self.pt1, self.pt2, self.pt3, self.pt4])
        pts_2d = self.lidar_project_to_image(pts_lidar.transpose(), self.proj_lidar2cam)
        
        src_pt = np.float32([[int(np.round(pts_2d[0][0])), int(np.round(pts_2d[1][0]))],
                             [int(np.round(pts_2d[0][1])), int(np.round(pts_2d[1][1]))],
                             [int(np.round(pts_2d[0][2])), int(np.round(pts_2d[1][2]))],
                             [int(np.round(pts_2d[0][3])), int(np.round(pts_2d[1][3]))]])

        self.resolution = 10 # 1pixel = 10cm
        self.grid_size = (int((self.pt1[1] - self.pt2[1]) * 100 / self.resolution), int((self.pt1[0] - self.pt3[0]) * 100 / self.resolution))

        dst_pt = np.float32([[0, 0], [self.grid_size[0], 0], [self.grid_size[0], self.grid_size[1]], [0, self.grid_size[1]]])
        self.M = cv2.getPerspectiveTransform(src_pt, dst_pt)

    def lidar_project_to_image(self, points, proj_mat):
        num_pts = points.shape[1]
        points = np.vstack((points, np.ones((1, num_pts))))
        points = np.dot(proj_mat, points)
        points[:2, :] /= points[2, :]
        return points[:2, :]

    def undistort(self, img):
        DIM = (img.shape[1], img.shape[0])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, np.eye(3), self.camera_matrix, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return undistorted_img

    def topview(self, img):
        topveiw_img = cv2.warpPerspective(img, self.M, (self.grid_size[0], self.grid_size[1]))
        return topveiw_img 