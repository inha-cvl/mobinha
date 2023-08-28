from numpy.linalg import norm
from cvxpy import Variable, Minimize, sum_squares, Problem
from math import sin, cos, atan2, radians, degrees
import numpy as np

class MPCController:
    def __init__(self, CP):
        self.L = CP.wheelbase
        self.N = 20  # 예측 지향점 개수
        self.dt = 0.1  # 타임 스텝

    def run(self, vEgo, path, position, yawRate):
        x = Variable(self.N)
        y = Variable(self.N)
        delta = Variable(self.N-1)

        # 초기 위치와 방향 설정
        x0 = position[0]
        y0 = position[1]
        yaw0 = radians(yawRate)

        objective = Minimize(sum_squares(x - path[:, 0]) + sum_squares(y - path[:, 1]))

        constraints = [x[0] == x0,
                       y[0] == y0]

        for i in range(self.N - 1):
            constraints += [x[i+1] == x[i] + vEgo * cos(yaw0 + delta[i]) * self.dt,
                            y[i+1] == y[i] + vEgo * sin(yaw0 + delta[i]) * self.dt]

        prob = Problem(objective, constraints)
        prob.solve()

        # 첫 번째 스티어링 각도를 반환
        return degrees(delta.value[0])