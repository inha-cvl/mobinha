import rospy
from std_msgs.msg import Float32
from numpy.linalg import norm
from math import sin, cos, atan2, radians, degrees
import numpy as np
from libs.interpolate import interpolate
import scipy.linalg as la


class PurePursuit:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, CP):
        # niro
        self.L = CP.wheelbase
        self.temp_lx = None
        self.temp_ly = None

        self.isLaneChange = False
        self.isBank = False
        rospy.Subscriber('/tmp_target_lfc', Float32, self.target_lfc_cb)
        rospy.Subscriber('/tmp_target_k', Float32, self.target_k_cb)
        rospy.Subscriber('/mobinha/avoid_gain', Float32, self.avoid_gain_cb)
        self.avoid_gain = 0.0

        self.k = CP.lateralTuning.lqr.k
        self.wheel_base = CP.wheelbase
        self.Lfc = CP.lateralTuning.lqr.l
        self.k_curva = 20.0

        self.cur_curvature = 0.0

    def target_lfc_cb(self, msg):
        self.Lfc = msg.data

    def target_k_cb(self, msg):
        self.k = msg.data

    def avoid_gain_cb(self, msg):
        self.avoid_gain = msg.data

    def lane_change_cb(self, msg):
        if msg.data == 1:
            self.isLaneChange = True
        else:
            self.isLaneChange = False

    def bank_cb(self, msg):
        if msg.data == 1:
            self.isBank = True
        else:
            self.isBank = False

    def euc_distance(self, pt1, pt2):
        return norm([pt2[0] - pt1[0], pt2[1] - pt1[1]])

    def find_nearest_idx(self, pts, pt):
        min_dist = float('inf')
        min_idx = 0
        for idx, pt1 in enumerate(pts):
            dist = self.euc_distance(pt1, pt)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        return min_idx

    def get_local(self, my_x, my_y, yaw, path):
        in_x, in_y = path
        new_x = (in_x - my_x) * cos(-yaw) - (in_y - my_y)*sin(-yaw)
        new_y = (in_x - my_x) * sin(-yaw) + (in_y - my_y)*cos(-yaw)
        return (new_x, new_y)

    def run(self, vEgo, path, position, yawRate, cte):
        lfd = self.Lfc+self.k*vEgo
        lfd = np.clip(lfd, 4, 60)
        steering_angle = 0.
        lx, ly = path[0]
        for point in path:
            diff = np.asarray((point[0]-position[0], point[1]-position[1]))
            rotation_matrix = np.array(
                ((np.cos(-radians(yawRate)), -np.sin(-radians(yawRate))), (np.sin(-radians(yawRate)),  np.cos(-radians(yawRate)))))
            rotated_diff = rotation_matrix.dot(diff)
            if rotated_diff[0] > 0:
                dis = np.linalg.norm(rotated_diff-np.array([0, 0]))
                if dis >= lfd:
                    theta = np.arctan2(rotated_diff[1]-self.avoid_gain, rotated_diff[0])
                    steering_angle = np.arctan2(2*self.L*np.sin(theta), lfd)
                    steering_angle = steering_angle + np.arctan2(0.1*cte, vEgo) if vEgo > 6 else steering_angle
                    lx = point[0]
                    ly = point[1]
                    break
        return degrees(steering_angle), (lx, ly)
    

    def run_experimental_rhc(self, vEgo, path, idxEgo, posEgo, yawEgo, cte, previous_steer):
        
        # 시스템 파라미터
        L = self.L  # 차량의 축간거리 (m)
        lookahead_distance = 6  # lookahead distance in meters
        resolution = 0.1  # path resolution in meters
        dt = 0.1  # 시간 간격 (s)
        T = 1.0  # 예측 시간 (s)
        N = int(T / dt)  # 예측 창의 시간 스텝 수

        # 현재 상태 업데이트
        xEgo, yEgo = posEgo
        psiEgo = yawEgo
        
        x0 = np.array([xEgo, yEgo, psiEgo])

        # 선형화된 시스템 매개변수 업데이트 함수
        def get_system_matrices(psi, delta):
            A = np.array([[0, 0, -vEgo * np.sin(psi)],
                        [0, 0, vEgo * np.cos(psi)],
                        [0, 0, 0]])
            B = np.array([[np.cos(psi), 0],
                        [np.sin(psi), 0],
                        [np.tan(delta) / L, vEgo / (L * np.cos(delta)**2)]])
            return A, B

        # LQR 가중치 행렬
        Q = np.diag([1, 1, 100])  # 상태 오차 가중치 (yaw에 더 큰 가중치를 부여)
        R = np.array([[0.1, 0.1]])  # 제어 입력 가중치 (작은 값을 설정하여 민감하게 반응)

        # 초기화
        steer = previous_steer  # 이전 조향각으로 초기화

        # 목표 yaw angle 설정
        lookahead_idx = int(lookahead_distance / resolution)
        target_idx = min(idxEgo + lookahead_idx, len(path) - 1)
        target_x, target_y = path[target_idx]

        # 목표 yaw angle 계산
        target_yaw = np.arctan2(target_y - yEgo, target_x - xEgo)

        # 선형화된 시스템 매개변수 업데이트
        delta = np.radians(steer)  # 이전 조향각 사용
        A, B = get_system_matrices(psiEgo, delta)

        if not self.check_controllability(A, B):
            print("A=", A)
            print("B=", B)
            print("System is not controllable.. PURE PURSUIT.")
            return self.run(vEgo, path, idxEgo, posEgo, yawEgo, cte
                            )
        
        try:
            # 연속 시간 Riccati 방정식을 풀어 P를 계산
            P = la.solve_continuous_are(A, B, Q, R)
            # 연속 시간 LQR 이득 행렬 계산
            K = np.linalg.inv(R) @ B.T @ P
        except np.linalg.LinAlgError:
            print("A=", A)
            print("B=", B)
            print("A is not inversible..")
            return self.run(vEgo, path, idxEgo, posEgo, yawEgo, cte)

        # 목표 상태 벡터 설정
        x_d = np.array([target_x, target_y, target_yaw])
        
        # 현재 상태에서 제어 입력 계산
        u = -K @ (x0 - x_d)
        delta = u[0]

        # 조향각 계산
        steer = np.degrees(delta)

        return steer, (target_x, target_y)


    def calc_idx(self, pt, path):
        min_dist = float('inf')
        min_idx = 0

        for idx, pt1 in enumerate(path):
            dist = np.sqrt((pt[0]-pt1[0])**2+(pt[1]-pt1[1])**2)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx

        if min_idx == len(path) - 1:
            pt1 = path[min_idx-1]
        else:
            pt1 = path[min_idx]

        return min_idx