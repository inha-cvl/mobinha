import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt

# 시스템 파라미터
L = 2.5  # 차량의 축간거리 (m)
v = 10  # 차량 속도 (m/s)
psi_d_list = np.deg2rad([20, 20, 20, 20, 20, 20, 20, 20, 20, 20])  # 예시 데이터
dt = 0.1  # 시간 간격 (s)
T = 1.0  # 예측 시간 (s)
N = int(T / dt)  # 예측 창의 시간 스텝 수

# 목표 yaw angle (타겟 리스트)
psi_d_time = np.arange(len(psi_d_list)) * dt  # 타겟 yaw angle에 해당하는 시간

# 이산 시간 시스템으로 변환 함수
def discretize_system(A, B, dt):
    Ad = la.expm(A * dt)
    Bd = np.zeros_like(B)
    tau = np.linspace(0, dt, 100)
    for i in range(len(tau) - 1):
        d_tau = tau[i+1] - tau[i]
        Bd += la.expm(A * tau[i]) @ B * d_tau
    return Ad, Bd

# LQR 가중치 행렬
Q = np.eye(3)  # 단위 행렬 (상태 오차 가중치)
R = np.array([[1]])  # 제어 입력 가중치

# 초기 상태
x0 = np.array([0, 0, 0])  # 초기 위치 및 각도

# 시뮬레이션 시간
t_final = len(psi_d_list) * dt
n_steps = int(t_final / dt)

# 상태 배열 초기화
x = np.zeros((3, n_steps))
x[:, 0] = x0

# 시뮬레이션 실행
for k in range(1, n_steps):
    # 예측 창 내의 목표 yaw angle 설정
    if k < len(psi_d_list):
        psi_d = psi_d_list[k]
    else:
        psi_d = psi_d_list[-1]
    
    # 선형화된 시스템 매개변수 업데이트
    A = np.array([[0, 0, -v * np.sin(psi_d)],
                  [0, 0, v * np.cos(psi_d)],
                  [0, 0, 0]])
    
    B = np.array([[0],
                  [0],
                  [v / (L * np.cos(psi_d)**2)]])
    
    # 이산 시간 시스템으로 변환
    Ad, Bd = discretize_system(A, B, dt)
    
    # 이산 시간 Riccati 방정식을 풀어 P를 계산
    P = la.solve_discrete_are(Ad, Bd, Q, R)
    
    # 이산 시간 LQR 이득 행렬 계산
    K = np.dot(la.inv(np.dot(np.dot(Bd.T, P), Bd) + R), np.dot(np.dot(Bd.T, P), Ad))
    
    # 목표 상태 벡터 설정
    x_d = np.array([0, 0, psi_d])
    
    # 현재 상태에서 제어 입력 계산
    u = -np.dot(K, (x[:, k-1] - x_d))
    delta = u[0]
    
    # 시스템 상태 업데이트
    x[0, k] = x[0, k-1] + v * np.cos(x[2, k-1]) * dt
    x[1, k] = x[1, k-1] + v * np.sin(x[2, k-1]) * dt
    x[2, k] = x[2, k-1] + (v / L) * np.tan(delta) * dt
    print(delta )
print(x[2])
# 결과 시각화
plt.figure()
plt.plot(x[0, :], x[1, :], label='후륜 중심 경로')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('후륜 중심 경로 추적')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
