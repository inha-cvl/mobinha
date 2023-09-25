#!/usr/bin/python
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Pose, Vector3
from selfdrive.visualize.rviz_utils import *
from selfdrive.control.libs.purepursuit import PurePursuit
from selfdrive.control.libs.stanley import StanleyController
from selfdrive.control.libs.pid import PID
import rospy
import time
import osqp

from scipy.linalg import inv
import cvxpy as cp
import numpy as np

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

HZ = 1/20

class MPCController:

    def __init__(self, CP):
        self.pid = PID(CP.longitudinalTuning)
        self.purepursuit = PurePursuit(CP)
        self.stanley = StanleyController(CP)
        self.steer_ratio = CP.steerRatio
        self.target_v = 0.0
        self.local_path = None
        self.l_idx = 0
        self.prev_steer = 0.0
        self.local_path_theta = None
        self.local_path_radius = None
        self.local_path_k = None
        self.previous_yawRate = None  # To store the previous yawRate
        self.previous_time = None  # To store the previous time stamp
        self.angular_v = 0

        rospy.Subscriber('/mobinha/planning/local_path', Marker, self.local_path_cb)
        rospy.Subscriber('/mobinha/planning/target_v', Float32, self.target_v_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        rospy.Subscriber('mobinha/planning/local_path_theta', Float32MultiArray, self.local_path_theta_cb)
        rospy.Subscriber('mobinha/planning/local_path_radius', Float32MultiArray, self.local_path_radius_cb)
        rospy.Subscriber('mobinha/planning/local_path_k', Float32MultiArray, self.local_path_k_cb)
        self.pub_target_actuators = rospy.Publisher('/mobinha/control/target_actuators', Vector3, queue_size=1)
        self.pub_lah = rospy.Publisher('/mobinha/control/look_ahead', Marker, queue_size=1, latch=True)

    def limit_steer_change(self, steer):
        #TODO:limit logic error need modified 
        # steer_diff = steer - self.prev_steer
        # if abs(steer_diff) > 10:
        #     steer = self.prev_steer + (10 if steer_diff > 0 else -10)
        # else:
        #     self.prev_steer = steer
        # self.prev_steer = steer
        return steer
    
    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def target_v_cb(self, msg):
        self.target_v = msg.data

    def lane_information_cb(self, msg):
        self.l_idx = msg.orientation.y
    
    def local_path_theta_cb(self, msg):
        self.local_path_theta = msg.data

    def local_path_radius_cb(self, msg):
        self.local_path_radius = msg.data
    
    def local_path_k_cb(self, msg):
        self.local_path_k = msg.data

    def calc_accel_brake_pressure(self, pid, cur_v, pitch):
        th_a = 4 # 0~20 * gain -> 0~100 accel
        th_b = 13 # 0~20 * gain -> 0~100 brake
        gain = 5
        val_data = max(-th_b, min(th_a, pid))
        if val_data > 0.:
            accel_val = val_data*gain
            brake_val = 0.0
        elif val_data <= 0.:
            accel_val = 0.0
            if (self.target_v > 0 and cur_v >= 1.5*KPH_TO_MPS):
                brake_val = -val_data*gain
            elif pitch < -2.5:
                brake_val = 45
            else:
                brake_val = 32
        
        return accel_val, brake_val
    
    def get_init_acuator(self):
        vector3 = Vector3()
        vector3.x = 0 #steer
        vector3.y = 0 #accel
        vector3.z = 32 #brakx
        return vector3 
    
    def estimate_theta(self, path, index):
        """
        Estimate the orientation theta based on two consecutive points in the path.
        
        Parameters:
        path (np.ndarray): The reference path [x, y]
        index (int): The index of the current point in the path
        
        Returns:
        float: The estimated orientation theta
        """
        point_current = path[index]
        point_next = path[index + 1] if index + 1 < len(path) else path[index]
        
        dx = point_next[0] - point_current[0]
        dy = point_next[1] - point_current[1]
        
        theta = np.arctan2(dy, dx)
    
        return theta
 

    def calc_proj_pose(self, p0, p1, p2):
        """
        Calculate the projected pose of point p0 on the line formed by points p1 and p2.
    
        Parameters:
        p0 (np.ndarray): The pose [x0, y0, theta0] of the vehicle
        p1 (np.ndarray): The pose [x1, y1, theta1] of the first point on the reference path
        p2 (np.ndarray): The pose [x2, y2, theta2] of the second point on the reference path

        Returns:
        np.ndarray: The projected pose [x, y, theta] of the vehicle on the line formed by p1 and p2
        """
        tol = 1e-4
        proj_pose = np.zeros(3)

        if abs(p2[0] - p1[0]) < tol:
            # If the slope of the line formed by p1 and p2 is very large (close to vertical)
            proj_pose[0] = p1[0]
            proj_pose[1] = p0[1] 
        elif abs(p2[1] - p1[1]) < tol:
            # If the slope of the line formed by p1 and p2 is very small (close to horizontal)
            proj_pose[0] = p0[0]
            proj_pose[1] = p1[1]
        else:
            # General case
            slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
            proj_pose[0] = (slope * (p0[1] - p1[1]) + slope * slope * p1[0] + p0[0]) / (slope * slope + 1)
            proj_pose[1] = slope * (proj_pose[0] - p1[0]) + p1[1]

        # Calculate the orientation theta of the projected pose
        dx = proj_pose[0] - p0[0]
        dy = proj_pose[1] - p0[1]
        proj_pose[2] = np.arctan2(dy, dx)

        return proj_pose

    # Correct the indexing issues and re-define the load_mpc_params function
    def load_mpc_params(self, veh_params):
        """
        Set up MPC parameters based on the vehicle parameters.
        
        Parameters:
        - veh_params: A dictionary containing vehicle parameters
        
        Returns:
        - mpc_params: A dictionary containing MPC parameters
        """
        
        # Given and calculated constants
        # mass_fl = mass_fr = mass_rl = mass_rr = 500  # in kg
        mass = 2245  # Total mass in kg
        mass_front = 1200  # Mass at the front
        mass_rear = 1045  # Mass at the rear
        
        # Constants
        eps = 0.01
        cutoff_freq = 10
        mean_filter_window_size = 10
        max_iteration = 150
        max_throttle_minimum_action = 0.0
        max_brake_minimum_action = 0.0
        max_lateral_acceleration = 5.0
        standstill_acceleration = -3.0
        unconstraint_control_diff_limit = 5.0
        
        # Calculate lf, lr, and iz
        lf = veh_params['wheel_base'] * (1.0 - mass_front / mass)
        lr = veh_params['wheel_base'] * (1.0 - mass_rear / mass)
        iz = lf * lf * mass_front + lr * lr * mass_rear

        # Constants related to the vehicle and matrices
        cf = 191788.56874
        cr = 202119.10055

        basic_state_size = 6
        control_size = 2
        
        # Matrix A and its coefficients
        matrix_a = np.zeros((basic_state_size, basic_state_size))
        matrix_a_coeff = np.zeros((basic_state_size, basic_state_size))
        matrix_a[0, 1] = 1.0
        matrix_a[1, 2] = (cf + cr) / mass
        matrix_a[2, 3] = 1.0
        matrix_a[3, 2] = (lf * cf - lr * cr) / iz
        matrix_a[4, 5] = 1.0
        matrix_a[5, 5] = 0.0
        
        matrix_a_coeff[1, 1] = -(cf + cr) / mass
        matrix_a_coeff[1, 3] = (lr * cr - lf * cf) / mass
        matrix_a_coeff[2, 3] = 1.0
        matrix_a_coeff[3, 1] = (lr * cr - lf * cf) / iz
        matrix_a_coeff[3, 3] = -1.0 * (lf * lf * cf + lr * lr * cr) / iz
        
        matrix_a[1, 1] = matrix_a_coeff[1, 1]
        matrix_a[1, 3] = matrix_a_coeff[1, 3]
        matrix_a[3, 1] = matrix_a_coeff[3, 1]
        matrix_a[3, 3] = matrix_a_coeff[3, 3]
        
        # Matrix B
        matrix_b = np.zeros((basic_state_size, control_size))
        matrix_b[1, 0] = cf / mass
        matrix_b[3, 0] = lf * cf / iz
        matrix_b[4, 1] = 0
        matrix_b[5, 1] = -1

        # Matrix C
        matrix_c = np.zeros((basic_state_size, 1))
        matrix_c[5, 0] = 1.0

        # Identity Matrix
        I = np.eye(basic_state_size)
        
        # Save the matrices and parameters
        mpc_params = {
            'A': matrix_a,
            'B': matrix_b,
            'C': matrix_c,
            'I': I,
            'ts': HZ,  # MPC time period
            'lr': lr,
            'lf': lf,
            'cf': cf,
            'cr': cr,
            'iz': iz,
            'mass': mass
        }
        
        return mpc_params
    
    def update_state_matrix(self, v, mpc_params):
        """
        Update the state matrix A, B, C based on the current vehicle speed and existing mpc_params.
        
        Parameters:
        - v: Current vehicle speed
        - mpc_params: Existing mpc_params containing A, B, C matrices
        
        Returns:
        - Updated mpc_params
        """
        
        EPSILON = 1e-9
        v = v if v != 0 else EPSILON

        # Extract existing matrices
        A = mpc_params['A']
        B = mpc_params['B']
        C = mpc_params['C']
        
        # Constants for updating matrices
        cf = mpc_params['cf']
        cr = mpc_params['cr']
        lf = mpc_params['lf']
        lr = mpc_params['lr']
        iz = mpc_params['iz']
        mass = mpc_params['mass']
        
        # Update the A matrix
        A[1, 1] = A[1,1]/v
        A[1, 3] = A[1,3]/v
        A[3, 1] = A[3,1]/v
        A[3, 3] = A[3,3]/v
        
        # Update the B matrix
        # B[1, 0] = v * cf / mass
        # B[3, 0] = v * lf * cf / iz
        
        # Update the C matrix
        C[1,0] = (lr*cr-lf*cf)/(mass*v)-v
        C[3,0] = -(lf*lf*cf+lr*lr*cr)/(iz*v)

        # Update mpc_params with the new matrices
        mpc_params['A'] = A
        # mpc_params['B'] = B
        mpc_params['C'] = C
        
        return mpc_params
    
    def angle_normalization(self, angle_origin):
        """
        Normalize the angle to the range [-pi, pi].
        
        Parameters:
        - angle_origin: The original angle
        
        Returns:
        - angle_norm: The normalized angle
        """
        if angle_origin > np.pi:
            angle_norm = angle_origin - 2 * np.pi
        elif angle_origin < -np.pi:
            angle_norm = angle_origin + 2 * np.pi
        else:
            angle_norm = angle_origin
        return angle_norm
    
    def solve_mpc_problem(self, A, B, C, matrix_q, matrix_r, lower, upper, ref_state, Np, Nc, matrix_state, control_state):
        # Initialize matrices
        q = A.shape[0]
        matrix_aa = np.zeros((q, Np * A.shape[1]))
        
        for i in range(Np):
            matrix_aa[:, i*q:(i+1)*q] = np.linalg.matrix_power(A, i + 1) # 6 * 60
        
        # matrix k: 60 * 20
        K_cell = [[None for _ in range(Np)] for _ in range(Np)]

        for r in range(Np):
            for c in range(Np):
                if c <= r:
                    K_cell[r][c] = np.linalg.matrix_power(A, r - c) @ B
                else:
                    K_cell[r][c] = np.zeros(B.shape)

        # Now, convert K_cell to a block matrix
        matrix_k = np.block(K_cell)
        
        c_r = C.shape[0]
        matrix_cc = np.zeros((Np * c_r, C.shape[1]))
        matrix_cc[0:c_r, 0:1] = C # 0 -> 0:1
        
        for j in range(1, Np):
            matrix_cc[j*c_r:(j+1)*c_r, 0:1] = np.linalg.matrix_power(A, j) @ C + matrix_cc[(j-1)*c_r:j*c_r, 0:1] # 0 -> 0:1
        
        matrix_Q = np.kron(np.eye(Np), matrix_q)
        matrix_R = np.kron(np.eye(Np), matrix_r)
        
        matrix_ll = np.tile(lower, (Np, 1))
        matrix_ll = matrix_ll.reshape(-1, 1)
        matrix_uu = np.tile(upper, (Np, 1))
        matrix_uu = matrix_uu.reshape(-1, 1)
        matrix_ll = matrix_ll.reshape(-1)
        matrix_uu = matrix_uu.reshape(-1)
        
        matrix_M = np.zeros((Np * q, 1))
        
        for i in range(Np):
            matrix_M[i*q:(i+1)*q, :] = np.linalg.matrix_power(A, i+1) @ matrix_state
        
        matrix_t = np.zeros((B.shape[0] * Np, 1))
        matrix_v = np.tile(control_state, (Np, 1))
        # print(matrix_k.shape)
        # print(matrix_Q.shape)
        # print(matrix_R.shape)
        matrix_m1 = matrix_k.T @ matrix_Q @ matrix_k + matrix_R
        matrix_m1 = (matrix_m1 + matrix_m1.T) / 2
        matrix_m2 = matrix_k.T @ matrix_Q @ (matrix_M + matrix_cc + matrix_t)

        # 여기에서 QR 문제를 풀면 됩니다. 이 부분은 생략하였습니다.
        # Format in qp_solver
        # min_x  : q(x) = 0.5 * x^T * Q * x  + x^T c
        # with respect to:  A * x = b (equality constraint)
        # C * x >= d (inequality constraint)
        # l_row, l_col = matrix_ll.shape  # or len(matrix_ll), len(matrix_ll[0])
        matrix_inequality_constrain_ll = np.eye(Np * Nc)
        # print("matrix_inequality_constrain_ll shape: ", matrix_inequality_constrain_ll.shape)
        # print("matrix_inequality_constrain_ll: ", matrix_inequality_constrain_ll)
        # u_row = len(matrix_uu)  # or matrix_uu.shape[0]
        matrix_inequality_constrain_uu = np.eye(Np * Nc, 1)
        # print("matrix_inequality_constrain_uu shape: ", matrix_inequality_constrain_uu.shape)
        # print("matrix_inequality_constrain_uu: ", matrix_inequality_constrain_uu)

        P = matrix_m1
        # print("P shape: ", P.shape)
        
        from scipy.sparse import csc_matrix

        P_sparse = csc_matrix(P)
        # print("P_sparse shape: ", P_sparse.shape)

        q = matrix_m2.ravel()
        A = matrix_inequality_constrain_ll
        # print("A shape: ", A.shape)
        A_sparse = csc_matrix(A)
        # print("A_sparse shape: ", A_sparse.shape)
        l = matrix_inequality_constrain_uu.ravel()
        # print("l shape: ", l.shape)
        # print("l:", l)
        u = matrix_uu.ravel()
        # print("u shape: ", u.shape)
        # print("u: ", u)

        # Variables
        x = cp.Variable(Np * Nc)

        # Objective function
        objective = cp.Minimize(0.5 * cp.quad_form(x, matrix_m1) + matrix_m2.T @ x)

        # Constraints
        constraints = [
            matrix_ll <= x,
            x <= matrix_uu
        ]

        # Solve the QP problem using cvxpy
        problem = cp.Problem(objective, constraints)
        problem.solve()

        # Extract control variables
        matrix_v = x.value
        return matrix_v[0], matrix_v[1]

        # m = osqp.OSQP()
        # m.setup(P=P_sparse, q=q, A=A_sparse, l=l, u=u, verbose=False)
        # results = m.solve()

        # matrix_v = results.x
        # return matrix_v[0], matrix_v[1]# command를 반환하면 됩니다.
        

    # Define the calc_mpc function based on the given MATLAB code
    def calc_mpc(self, trajref, delta_x, veh_pose, ref_pose, mpc_params, index, veh_params):
        """
        Calculate the MPC control commands.
        
        Parameters:
        - trajref: Reference trajectory
        - delta_x: Delta between vehicle and reference poses
        - veh_pose: Vehicle pose
        - ref_pose: Reference pose
        - mpc_params: MPC parameters
        - index: Index in the reference trajectory
        - veh_params: Vehicle parameters
        
        Returns:
        - steer_cmd: Steering command
        - acc: Acceleration
        - steer_feedforward: Feedforward steering angle
        """
        
        # Update state
        basic_state_size = 6
        matrix_state = np.zeros((basic_state_size, 1))
        control_state = np.zeros((2, 1))
        
        # Update parameter state
        dx, dy, dtheta = delta_x
        theta_des = trajref[index][2]
        theta = veh_pose[2]
        radius_des = trajref[index][3]
        k = trajref[index][4]  # curvature
        one_min_k = 1 - k
        one_min_k = 0.01 if one_min_k <= 0 else one_min_k
        
        v = veh_params['velocity']
        v_des = veh_params['v_des']
        angular_v = veh_params['angular_v']
        angular_v_des = v_des / radius_des if radius_des >= 0.01 else 0
        
        # Calculate the state
        # lateral error
        matrix_state[0, 0] = dy * np.cos(theta_des) - dx * np.sin(theta_des)
        # print("lateral error: ", matrix_state[0, 0])
        # lateral error rate
        matrix_state[1, 0] = v * np.sin(dtheta)
        # print("lateral error rate: ", matrix_state[1, 0])
        # heading error        
        matrix_state[2, 0] = self.angle_normalization(dtheta)
        # print("heading error: ", matrix_state[2, 0])
        # heading error rate
        heading_error_rate = angular_v - angular_v_des
        matrix_state[3, 0] = heading_error_rate
        # print("heading error rate: ", matrix_state[3, 0])
        # station error
        matrix_state[4, 0] = -(dx * np.cos(theta_des) + dy * np.sin(theta_des))
        # print("station error: ", matrix_state[4, 0])
        # speed error
        matrix_state[5, 0] = v_des - v * np.cos(dtheta) / one_min_k
        # print("speed error: ", matrix_state[5, 0])
        
        # Update Matrix
        mpc_params = self.update_state_matrix(v, mpc_params)
        A = mpc_params['A']
        B = mpc_params['B']
        C = mpc_params['C']
        I = np.eye(basic_state_size)
        ts = mpc_params['ts']
        
        # Discretization
        matrix_ad = np.dot((I + ts * 0.5 * A), np.linalg.inv(I - ts * 0.5 * A))
        matrix_bd = B * ts
        matrix_cd = C * ts * heading_error_rate
        
        # Feedforward angle Update
        kv = mpc_params['lr'] * mpc_params['mass'] / 2 / mpc_params['cf'] / veh_params['wheel_base'] \
        - mpc_params['lf'] * mpc_params['mass'] / 2 / mpc_params['cr'] / veh_params['wheel_base']
        steer_feedforward = np.arctan(veh_params['wheel_base'] * k + kv * v * v * k)
        steer_feedforward = self.angle_normalization(steer_feedforward)
        
        Np = 10
        Nc = 2
        
        lower_bound = np.zeros((Nc, 1))
        lower_bound[0, 0] = -veh_params['max_steer_angle']
        lower_bound[1, 0] = -veh_params['max_deceleration']

        upper_bound = np.zeros((Nc, 1))
        upper_bound[0, 0] = veh_params['max_steer_angle']
        upper_bound[1, 0] = veh_params['max_acceleration']

        matrix_q = np.zeros((basic_state_size, basic_state_size))
        matrix_q[0, 0] = 0.13
        matrix_q[2, 2] = 1

        matrix_r = 7 * np.eye(Nc, Nc)

        ref_state = np.zeros((basic_state_size, 1))

        steer_cmd, acc = self.solve_mpc_problem(matrix_ad, matrix_bd, matrix_cd,
                                matrix_q, matrix_r, lower_bound, upper_bound,
                                ref_state, Np, Nc,
                                matrix_state, control_state)
        gain = 0
        print("output : ", steer_cmd+gain, acc)
        
        return steer_cmd+gain, acc, steer_feedforward

    def calculate_angular_velocity(self, current_heading_deg, previous_heading_deg, delta_time):
        # Convert degrees to radians
        current_heading_rad = current_heading_deg * (np.pi / 180)
        previous_heading_rad = previous_heading_deg * (np.pi / 180)
        
        # Calculate angular velocity in rad/ssteer_command -= 1
        angular_velocity_rad_per_s = (current_heading_rad - previous_heading_rad) / delta_time
        
        # Handle the wrap-around issue at the -180 to 180 boundary
        if angular_velocity_rad_per_s > np.pi:
            angular_velocity_rad_per_s -= 2 * np.pi
        elif angular_velocity_rad_per_s < -np.pi:
            angular_velocity_rad_per_s += 2 * np.pi
        
        return angular_velocity_rad_per_s

    # limit_steer_angle function
    def limit_steer_angle(self,steer_angle, max_steer_angle):
        steer_angle = min(steer_angle, max_steer_angle)
        steer_angle = max(steer_angle, -max_steer_angle)
        return steer_angle

    # limit_steer_by_angular_vel function
    def limit_steer_by_angular_vel(self,expect_steer_angle, current_steer_angle, max_angular_vel, time_step):
        steer_angle = expect_steer_angle
        max_steer_angle = current_steer_angle + max_angular_vel * time_step
        min_steer_angle = current_steer_angle - max_angular_vel * time_step
        steer_angle = min(steer_angle, max_steer_angle)
        steer_angle = max(steer_angle, min_steer_angle)
        return steer_angle

    def run(self, sm):
        CS = sm.CS
        vector3 = self.get_init_acuator()

        if self.local_path != None:
            current_time = time.time()  # Assuming sm.timestamp gives the current time in seconds
            current_yawRate = CS.yawRate  # Assuming CS.yawRate gives the current heading in degrees

            veh_pose = (CS.position.x, CS.position.y, CS.yawRate*np.pi/180)

            min_length = min(len(self.local_path), len(self.local_path_theta), len(self.local_path_radius), len(self.local_path_k))
            idx = int(self.l_idx)
            print("idx : ",idx)
            # Trim the arrays to have the same length
            self.local_path = self.local_path[:min_length]
            self.local_path_theta = self.local_path_theta[:min_length]
            self.local_path_radius = self.local_path_radius[:min_length]
            self.local_path_k = self.local_path_k[:min_length]

            trajref = np.column_stack((self.local_path, self.local_path_theta, self.local_path_radius, self.local_path_k))
            # trajref = trajref[::4]
            # print("trajref : ",trajref[int(self.l_idx):int(self.l_idx)+10])
            ref_pose = trajref[idx][0:3]
            
            delta_x = veh_pose - ref_pose
            print("veh_pose : ",veh_pose)
            print("ref_pose : ",ref_pose)
            delta_x[2] = self.angle_normalization(delta_x[2])
            print("delta_x : ",delta_x)
            
            if self.previous_yawRate is not None and self.previous_time is not None:
                delta_time = current_time - self.previous_time
                self.angular_v = self.calculate_angular_velocity(current_yawRate, self.previous_yawRate, delta_time) # rad/s

            # print("angular_v : ", self.angular_v)

            veh_params = {
            'velocity': CS.vEgo,  # Current velocity, m/s
            'v_des': self.target_v,  # Desired velocity, m/s
            'angular_v': self.angular_v,  # Current angular velocity, rad/s
            'wheel_base': 3.0,  # Wheelbase, m
            'max_steer_angle': 30 / 180 * np.pi,  # Maximum steer angle, rad
            'max_angular_vel': 5 / 180 * np.pi,  # Maximum angular velocity, rad/s
            'max_acceleration': 3,  # Maximum acceleration
            'max_deceleration': 10,  # Maximum deceleration
            'vehicle_size': 20,
            'vehicle_length': 6 * 0.1 * 0.8  # velocity * time_step * 0.8, assuming time_step = 0.1 for now
            }

            mpc_params = self.load_mpc_params(veh_params)
            start_time = time.time()  # Start time
            steer_command, acc, steer_feedforward = self.calc_mpc(trajref, delta_x, veh_pose, ref_pose, mpc_params, idx, veh_params)
            end_time = time.time()  # End time
            mpc_execution_time = end_time - start_time
            print("MPC execution time: ", mpc_execution_time)
            steer_cmd = steer_command# + steer_feedforward # steer cmd is degree unit
            # print("steer_cmd : ",steer_cmd)
            # steer_cmd = self.limit_steer_by_angular_vel(steer_cmd, CS.actuators.steer/self.steer_ratio/180*np.pi, veh_params['max_angular_vel'], HZ)
            # print("limit_steer_cmd_by_angular_vel : ",steer_cmd)
            steer = self.limit_steer_angle(steer_cmd, veh_params['max_steer_angle'])
            print("limit steer : " ,steer)
            steer = steer * 180 / np.pi # degree unit
            print("steer deg : ",steer)
            steer = steer * self.steer_ratio # steering
            # print("steer wheel deg: ",steer)
            print("==================================")
            pid = self.pid.run(self.target_v, CS.vEgo) #-100~100
            accel, brake = self.calc_accel_brake_pressure(pid, CS.vEgo, CS.pitchRate)
            
            vector3.x = steer
            vector3.y = accel
            vector3.z = brake

            self.previous_yawRate = current_yawRate
            self.previous_time = current_time

        if CS.cruiseState != 1:
            vector3.x = CS.actuators.steer
            vector3.y = CS.actuators.accel
            vector3.z = CS.actuators.brake

        self.pub_target_actuators.publish(vector3)
