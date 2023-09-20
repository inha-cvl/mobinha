import numpy as np
import osqp

def solve_mpc_problem(A, B, C, matrix_q, matrix_r, lower, upper, ref_state, horizon, control, matrix_state, control_state):
    # Initialize matrices
    q = A.shape[0]
    matrix_aa = np.zeros((q, horizon * A.shape[1]))
    
    for i in range(horizon):
        matrix_aa[:, i*q:(i+1)*q] = np.linalg.matrix_power(A, i + 1) # 6 * 60
    
    # matrix k: 60 * 20
    K_cell = np.zeros((horizon, horizon, *B.shape))
    for r in range(horizon):
        for c in range(horizon):
            if c <= r:
                K_cell[r, c] = np.linalg.matrix_power(A, r - c) @ B
            else:
                K_cell[r, c] = np.zeros_like(B)
                
    matrix_k = np.block(K_cell.tolist())
    
    c_r = C.shape[0]
    matrix_cc = np.zeros((horizon * c_r, C.shape[1]))
    matrix_cc[0:c_r, 0] = C
    
    for j in range(1, horizon):
        matrix_cc[j*c_r:(j+1)*c_r, 0] = np.linalg.matrix_power(A, j) @ C + matrix_cc[(j-1)*c_r:j*c_r, 0]
    
    matrix_Q = np.kron(np.eye(horizon), matrix_q)
    matrix_R = np.kron(np.eye(horizon), matrix_r)
    
    matrix_ll = np.tile(lower, (horizon, 1))
    matrix_uu = np.tile(upper, (horizon, 1))
    
    matrix_M = np.zeros((horizon * q, 1))
    
    for i in range(horizon):
        matrix_M[i*q:(i+1)*q, :] = np.linalg.matrix_power(A, i+1) @ matrix_state
    
    matrix_t = np.zeros((B.shape[0] * horizon, 1))
    matrix_v = np.tile(control_state, (horizon, 1))
    
    matrix_m1 = matrix_k.T @ matrix_Q @ matrix_k + matrix_R
    matrix_m1 = (matrix_m1 + matrix_m1.T) / 2
    matrix_m2 = matrix_k.T @ matrix_Q @ (matrix_M + matrix_cc + matrix_t)

    # 여기에서 QR 문제를 풀면 됩니다. 이 부분은 생략하였습니다.

    l_row, l_col = matrix_ll.shape  # or len(matrix_ll), len(matrix_ll[0])
    matrix_inequality_constrain_ll = np.eye(horizon * control)
    u_row = len(matrix_uu)  # or matrix_uu.shape[0]
    matrix_inequality_constrain_uu = np.eye(horizon * control, 1)

    P = matrix_m1
    q = matrix_m2.ravel()
    A = matrix_inequality_constrain_ll
    l = matrix_inequality_constrain_uu.ravel()
    u = matrix_uu.ravel()

    m = osqp.OSQP()
    m.setup(P=P, q=q, A=A, l=l, u=u)
    results = m.solve()

    matrix_v = results.x
    return matrix_v[0], matrix_v[1]# command를 반환하면 됩니다.

# 예제로 사용할 변수들을 정의하고 함수를 호출할 수 있습니다.
# A, B, C, matrix_q, matrix_r 등을 먼저 정의해주세요.
# command = solve_mpc_problem(A, B, C, matrix_q, matrix_r, lower, upper, ref_state, horizon, control, matrix_state, control_state)


    def solve_mpc_problem(self, A, B, C, Q, R, lower_bound, upper_bound, ref, horizon, control_horizon, x0, u0):
        n = A.shape[0]
        m = B.shape[1]
        # print(A, B)
        # print(ref,C)
        ref = ref.flatten()
        C = C.flatten()
        # print(ref,C)
        x = cp.Variable((n, horizon+1))
        u = cp.Variable((m, control_horizon))
        # print(x,u)
        cost = 0
        constraints = []
        for t in range(horizon):
            if t < control_horizon:
                cost += cp.quad_form(x[:, t] - ref, Q) + cp.quad_form(u[:, t], R)
                constraints += [x[:, t+1] == A @ x[:, t] + B @ u[:, t] + C,
                                u[:, t] >= lower_bound,
                                u[:, t] <= upper_bound]
            else:
                cost += cp.quad_form(x[:, t] - ref, Q)
                constraints += [x[:, t+1] == A @ x[:, t] + C]
        constraints += [x[:, 0] == x0.flatten()]
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()
        # print(problem.status)
        # print(problem.value)
        # print(u[:, 0].value)
        if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
            u1 = u[:, 0].value
            print(u1)
            return u1
        else:
            return np.zeros((m,))