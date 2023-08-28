from numpy.linalg import norm
from scipy.linalg import solve_continuous_are
from math import sin, cos, atan2, radians, degrees
import numpy as np

class LQRController:
    def __init__(self, CP):
        self.L = CP.wheelbase
        self.A = np.array([[0, 1], [0, -2 / 20.0]])  # 20.0은 예시로 사용한 차량 속도입니다. 실제 속도를 사용해야 합니다.
        self.B = np.array([[0], [1]])
        self.Q = np.array([[1, 0], [0, 1]])
        self.R = np.array([[1]])

    def calculate_lqr_gain(self):
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def lqr_control(self, cte, heading_error):
        x = np.array([[cte], [heading_error]])
        K = self.calculate_lqr_gain()
        delta = -K @ x
        return delta[0, 0]

    def run(self, vEgo, path, position, yawRate):
        # cte와 heading_error 계산 로직은 별도로 구현해야 합니다.
        cte = 0
        heading_error = 0

        lqr_steering_angle = self.lqr_control(cte, heading_error)

        return degrees(lqr_steering_angle)