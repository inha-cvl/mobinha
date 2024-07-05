import numpy as np
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt

# 시스템 파라미터
L = 3  # 차량의 축간거리 (m)
v = 1  # 차량 속도 (m/s)
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

# 목표 경로 생성
target_path = np.zeros((3, n_steps))
for k in range(n_steps):
    target_path[0, k] = v * np.cos(psi_d) * k * dt
    target_path[1, k] = v * np.sin(psi_d) * k * dt
    target_path[2, k] = psi_d

# 시뮬레이션 실행
for k in range(1, n_steps):
    u = control_law(x[:, k-1], x_d)
    delta = u[0]
    if k < 10:  # 처음 몇 번의 delta 값을 출력해보기
        print(f"Time: {k*dt:.2f}s, Delta: {delta:.4f}")

    x[0, k] = x[0, k-1] + v * np.cos(x[2, k-1]) * dt
    x[1, k] = x[1, k-1] + v * np.sin(x[2, k-1]) * dt
    x[2, k] = x[2, k-1] + (v / L) * np.tan(delta) * dt

    print(f"Time: {k*dt:.2f}s, Target Yaw: {target_path[2, k]:.2f} rad, Actual Yaw: {x[2, k]:.2f} rad")

# 경로 결과 시각화
plt.figure()
plt.plot(x[0, :], x[1, :], label='Actual Path')
plt.plot(target_path[0, :], target_path[1, :], 'r--', label='Target Path')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Path Tracking')
plt.legend()
plt.grid(True)
plt.axis('equal')

# Yaw 각도 비교 시각화
plt.figure()
plt.plot(np.arange(n_steps) * dt, x[2, :], label='Actual Yaw')
plt.plot(np.arange(n_steps) * dt, target_path[2, :], 'r--', label='Target Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (rad)')
plt.title('Yaw Angle Comparison')
plt.legend()
plt.grid(True)

plt.show()
