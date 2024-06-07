from math import sin, cos, atan2, radians, degrees
import numpy as np
import scipy.linalg as la
from scipy.signal import cont2discrete

class PurePursuit:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, path):
        print("Pure Pursuit model initiated!")
        self.L = 3
        self.prev_angle = None
        self.path = path
        self.yaw_list = []
        for i in range(1, len(self.path)):
            dx = self.path[i][0] - self.path[i-1][0]
            dy = self.path[i][1] - self.path[i-1][1]
            yaw = atan2(dy, dx)
            self.yaw_list.append(yaw)
        

    def run(self, vEgo, path, idx, position, yawRate, cte):
        # lfd = 1.8 # for speed 8km, factor 1
        # lfd = 2 # for speed 10
        # lfd = 4 # for speed 20, factor 1.2
        lfd = 6 # for speed 24, factor 1.2  
        # lfd = 6 # for speed 30, factor 1.4
        # lfd = 7 # for speed 40, factor 1.3
        # lfd = 10 # for speed 50, factor 1.3


        # print(f"CTE:{cte:.2f}")
        lfd = np.clip(lfd, 4, 60)  
        steering_angle = 0.  
        lx, ly = path[0] 
        
        for point in path[idx:]:
            diff = np.asarray((point[0]-position[0], point[1]-position[1]))
            rotation_matrix = np.array(
                ((np.cos(-radians(yawRate)), -np.sin(-radians(yawRate))), (np.sin(-radians(yawRate)),  np.cos(-radians(yawRate)))))
            rotated_diff = rotation_matrix.dot(diff)
            dis = np.linalg.norm(rotated_diff-np.array([0, 0]))
            if dis >= lfd: 
                theta = np.arctan2(rotated_diff[1], rotated_diff[0]) 
                steering_angle = np.arctan2(2*self.L*np.sin(theta), lfd)
                steering_angle = steering_angle + np.arctan2(0.1 * cte, vEgo) if vEgo > 6 else steering_angle
                lx = point[0]  
                ly = point[1]  
                break

        self.yaw_list.append(0)


        factor = 1.2

        return degrees(factor*steering_angle), (lx, ly) 
    

    def run_experimental(self, vEgo, path, idxEgo, posEgo, yawEgo, cte):
        # lfd = 1.8 # for speed 8km, factor 1
        # lfd = 2 # for speed 10
        # lfd = 4 # for speed 20, factor 1.2
        lfd = 6 # for speed 24, factor 1.2  
        # lfd = 6 # for speed 30, factor 1.4
        # lfd = 7 # for speed 40, factor 1.3
        # lfd = 10 # for speed 50, factor 1.3


        print(f"CTE:{cte:.2f}")
        heading = yawEgo
        x, y = posEgo

        resolution = 0.1
        lookahead_dis = 6 # [m]
        lookahead_idx = int(lookahead_dis/resolution)

        # Target index 결정
        target_idx = min(idxEgo + lookahead_idx, len(path)-1)
        if target_idx >= len(path)-1:
            target_idx = len(path)-1

        # 각도 계산
        target_x, target_y = path[target_idx]
        tmp = degrees(atan2(target_y - y, target_x - x)) % 360
        alpha = tmp - heading
        angle = atan2(2.0 * self.L * sin(radians(alpha)), lookahead_dis)
        steer = degrees(angle) 


        return steer, (target_x, target_y)

    def check_controllability(self, A, B):
        n = A.shape[0]
        controllability_matrix = B
        for i in range(1, n):
            controllability_matrix = np.hstack((controllability_matrix, np.linalg.matrix_power(A, i) @ B))
        rank_of_controllability_matrix = np.linalg.matrix_rank(controllability_matrix)
        if rank_of_controllability_matrix == n:
            return True
        else:
            return False

    def run_experimental_rhc(self, vEgo, path, idxEgo, posEgo, yawEgo, cte, steer):
        path = self.path # 이거 되면 나머지 변수명도 바꾸기
        # 시스템 파라미터
        L = self.L  # 차량의 축간거리 (m)
        lookahead_distance = 6  # lookahead distance in meters
        resolution = 0.1  # path resolution in meters
        dt = 0.1  # 시간 간격 (s)
        T = 1.0  # 예측 시간 (s)
        N = int(T / dt)  # 예측 창의 시간 스텝 수

        # 현재 상태
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
        R = np.diag([0.1, 0.1])  # 제어 입력 가중치 (작은 값을 설정하여 민감하게 반응)

        # 초기화
        target_x, target_y = path[idxEgo]

        for _ in range(N):
            # 목표 yaw angle 설정
            lookahead_idx = int(lookahead_distance)
            target_idx = min(idxEgo + lookahead_idx, len(path) - 1)
            target_x, target_y = path[target_idx]

            # 목표 yaw angle 계산
            target_yaw = self.yaw_list[idxEgo]

            # 선형화된 시스템 매개변수 업데이트
            delta = steer  # 현재 조향각 사용
            A, B = get_system_matrices(psiEgo, delta)

            if not self.check_controllability(A, B):
                print("System is not controllable. Switching to PURE PURSUIT.")
                return self.run(vEgo, path, idxEgo, posEgo, yawEgo, cte)
            
            try:
                P = la.solve_continuous_are(A, B, Q, R)
                K = np.linalg.inv(R) @ B.T @ P
            except np.linalg.LinAlgError:
                print("R is not inversible. Switching to PURE PURSUIT.")
                return self.run(vEgo, path, idxEgo, posEgo, yawEgo, cte)

            # 목표 상태 벡터 설정
            x_d = np.array([target_x, target_y, target_yaw])
            
            # 현재 상태에서 제어 입력 계산
            u = -K @ (x0 - x_d)
            delta = u[0]
            
            # 시스템 상태 업데이트
            xEgo = x0[0] + vEgo * np.cos(x0[2]) * dt
            yEgo = x0[1] + vEgo * np.sin(x0[2]) * dt
            psiEgo = x0[2] + (vEgo / L) * np.tan(delta) * dt
            x0 = np.array([xEgo, yEgo, psiEgo])

            # 조향각 계산
            steer = np.degrees(delta)

        return steer, (target_x, target_y)
