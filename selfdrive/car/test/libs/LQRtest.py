import numpy as np
import matplotlib.pyplot as plt

# 시스템 파라미터
L = 3  # 차량의 축간거리 (m)
v = 10  # 차량 속도 (m/s)
psi_d = np.deg2rad(290)  # 목표 헤딩 (라디안 단위로 변환)

# 시간 간격 및 시뮬레이션 시간
dt = 0.01
t_final = 100
n_steps = int(t_final / dt)

# 초기 상태
x0 = np.array([0, 0, np.pi/2])  # 초기 위치 및 각도

# 상태 배열 초기화
x = np.zeros((3, n_steps))
x[:, 0] = x0
x_d = np.array([0, 0, psi_d])

# 목표 경로 생성
target_path_x = np.linspace(0, v * np.cos(psi_d) * t_final, n_steps)
target_path_y = np.linspace(0, v * np.sin(psi_d) * t_final, n_steps)

# 제어 입력 계산 (최소자승법)
def least_squares_control(x, psi_d):
    A = np.array([[1, 0, -v * np.sin(psi_d)*dt],
                  [0, 1, v * np.cos(psi_d)*dt],
                  [0, 0, 1]])
    B = np.array([[0],
                  [0],
                  [v / (L * np.cos(psi_d)**2)]])
    
    # 상태 예측
    x_pred = A @ x
    
    # 최소자승 문제 정의
    H = B.T @ B
    
    f = -2 * B.T * (psi_d - x_pred[2])
    
    # 제어 입력 계산 (최소자승해)
    delta = -np.linalg.inv(H) @ f
    
    return delta[0][2]

# 시뮬레이션 실행
for k in range(1, n_steps):
    delta = least_squares_control(x[:, k-1], psi_d)
    x[0, k] = x[0, k-1] + v * np.cos(x[2, k-1]) * dt
    x[1, k] = x[1, k-1] + v * np.sin(x[2, k-1]) * dt
    x[2, k] = x[2, k-1] + (v / L) * np.tan(delta) * dt

# 결과 시각화
plt.figure()
plt.plot(x[0, :], x[1, :], label='후륜 중심 경로')
plt.plot(target_path_x, target_path_y, 'r--', label='목표 경로')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('후륜 중심 경로 추적')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
