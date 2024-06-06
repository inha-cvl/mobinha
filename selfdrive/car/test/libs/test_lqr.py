import numpy as np
from scipy.linalg import solve_continuous_are

# 시스템 파라미터
L = 3  # 차량의 축간거리 (m)
v = 10  # 차량 속도 (m/s)
psi_d = np.deg2rad(134)  # 타겟 yaw angle (라디안 단위로 변환)

# 선형화된 시스템 매개변수
A = np.array([[0, 0, -v * np.sin(psi_d)],
              [0, 0, v * np.cos(psi_d)],
              [0, 0, 0]])

B = np.array([[0],
              [0],
              [v / (L * np.cos(psi_d)**2)]])

# LQR 가중치 행렬
Q = np.diag([0, 0, 1])  # 상태 오차 가중치
R = np.array([[1]])     # 제어 입력 가중치

# Riccati 방정식을 풀어 P를 계산
P = solve_continuous_are(A, B, Q, R)

# LQR 이득 행렬 계산
K = np.linalg.inv(R) @ B.T @ P

# 초기 상태
x0 = np.array([0, 0, 0])  # 초기 위치 및 각도

# 상태 피드백 제어
def control_law(x, x_d):
    return -K @ (x - x_d)

# 시간 간격 및 시뮬레이션 시간
dt = 0.1
t_final = 10
n_steps = int(t_final / dt)

# 상태 배열 초기화
x = np.zeros((3, n_steps))
x[:, 0] = x0
x_d = np.array([0, 0, psi_d])

# 시뮬레이션 실행
for k in range(1, n_steps):
    u = control_law(x[:, k-1], x_d)
    delta = u[0]
    x[0, k] = x[0, k-1] + v * np.cos(x[2, k-1]) * dt
    x[1, k] = x[1, k-1] + v * np.sin(x[2, k-1]) * dt
    x[2, k] = x[2, k-1] + (v / L) * np.tan(delta) * dt
print(x[2])

# 결과 시각화
import matplotlib.pyplot as plt

plt.plot(x[0, :], x[1, :], label='후륜 중심 경로')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('후륜 중심 경로 추적')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
