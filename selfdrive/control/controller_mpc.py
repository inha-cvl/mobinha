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

from scipy.linalg import inv
import cvxpy as cp
import numpy as np

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


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
            'ts': 0.01,  # MPC time period
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
        A[1, 1] = -v * (cf + cr) / mass
        A[1, 3] = v * (lr * cr - lf * cf) / mass
        A[3, 1] = v * (lr * cr - lf * cf) / iz
        A[3, 3] = -v * (lf * lf * cf + lr * lr * cr) / iz
        
        # Update the B matrix
        B[1, 0] = v * cf / mass
        B[3, 0] = v * lf * cf / iz
        
        # Update the C matrix
        # No updates specified for C in the MATLAB code
        
        # Update mpc_params with the new matrices
        mpc_params['A'] = A
        mpc_params['B'] = B
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
        matrix_state[0, 0] = dy * np.cos(theta_des) - dx * np.sin(theta_des)
        matrix_state[1, 0] = v * np.sin(dtheta)
        matrix_state[2, 0] = self.angle_normalization(dtheta)
        matrix_state[3, 0] = angular_v - angular_v_des
        matrix_state[4, 0] = -(dx * np.cos(theta_des) + dy * np.sin(theta_des))
        matrix_state[5, 0] = v_des - v * np.cos(dtheta) / one_min_k
        
        # Update Matrix
        mpc_params = self.update_state_matrix(v, mpc_params)
        A = mpc_params['A']
        B = mpc_params['B']
        C = mpc_params['C']
        I = np.eye(basic_state_size)
        ts = mpc_params['ts']
        
        # Discretization
        matrix_ad = inv(I - ts * 0.5 * A) @ (I + ts * 0.5 * A)
        matrix_bd = B * ts
        matrix_cd = C * ts * (angular_v - angular_v_des)
        
        # Feedforward angle Update
        kv = mpc_params['lr'] * mpc_params['mass'] / 2 / mpc_params['cf'] / veh_params['wheel_base'] \
        - mpc_params['lf'] * mpc_params['mass'] / 2 / mpc_params['cr'] / veh_params['wheel_base']
        steer_feedforward = np.arctan(veh_params['wheel_base'] * k + kv * v * v * k)
        steer_feedforward = self.angle_normalization(steer_feedforward)
        
        horizon = 10
        control_horizon = 2
        lower_bound = np.zeros((control_horizon, 1))
        lower_bound[0, 0] = -veh_params['max_steer_angle']
        lower_bound[1, 0] = -veh_params['max_deceleration']
        upper_bound = np.zeros((control_horizon, 1))
        upper_bound[0, 0] = veh_params['max_steer_angle']
        upper_bound[1, 0] = veh_params['max_acceleration']
        matrix_q = np.zeros((basic_state_size, basic_state_size))
        matrix_q[0, 0] = 0.05
        matrix_q[2, 2] = 1.0
        matrix_r = np.eye(control_horizon, control_horizon)
        ref_state = np.zeros((basic_state_size, 1))

        # steer_cmd, acc = self.solve_mpc_problem(mpc_params['A'], mpc_params['B'], mpc_params['C'],
        #                               matrix_q, matrix_r,
        #                               lower_bound.flatten(), upper_bound.flatten(),
        #                               ref_state, horizon, control_horizon,
        #                               matrix_state, control_state)
        steer_cmd, acc = self.solve_mpc_problem(matrix_ad, matrix_bd, matrix_cd,
                                matrix_q, matrix_r,
                                lower_bound.flatten(), upper_bound.flatten(),
                                ref_state, horizon, control_horizon,
                                matrix_state, control_state)
        
        return steer_cmd, acc, steer_feedforward
    
    def calculate_angular_velocity(self, current_heading_deg, previous_heading_deg, delta_time):
        # Convert degrees to radians
        current_heading_rad = current_heading_deg * (np.pi / 180)
        previous_heading_rad = previous_heading_deg * (np.pi / 180)
        
        # Calculate angular velocity in rad/s
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

            veh_pose = (CS.position.x, CS.position.y, CS.yawRate)

            min_length = min(len(self.local_path), len(self.local_path_theta), len(self.local_path_radius), len(self.local_path_k))

            # Trim the arrays to have the same length
            self.local_path = self.local_path[:min_length]
            self.local_path_theta = self.local_path_theta[:min_length]
            self.local_path_radius = self.local_path_radius[:min_length]
            self.local_path_k = self.local_path_k[:min_length]

            trajref = np.column_stack((self.local_path, self.local_path_theta, self.local_path_radius, self.local_path_k))

            ref_pose = self.calc_proj_pose(veh_pose, trajref[int(self.l_idx)], trajref[int(self.l_idx)+1])
            delta_x = veh_pose - ref_pose
            
            if self.previous_yawRate is not None and self.previous_time is not None:
                delta_time = current_time - self.previous_time
                self.angular_v = self.calculate_angular_velocity(current_yawRate, self.previous_yawRate, delta_time)

            veh_params = {
            'velocity': CS.vEgo,  # Current velocity, m/s
            'v_des': CS.vEgo,  # Desired velocity, m/s
            'angular_v': self.angular_v,  # Current angular velocity, rad/s
            'wheel_base': 3.0,  # Wheelbase, m
            'max_steer_angle': 33 / 180 * np.pi,  # Maximum steer angle, rad
            'max_angular_vel': 6 / 180 * np.pi,  # Maximum angular velocity, rad/s
            'max_acceleration': 3,  # Maximum acceleration
            'max_deceleration': 3,  # Maximum deceleration
            'vehicle_size': 20,
            'vehicle_length': 6 * 0.1 * 0.8  # velocity * time_step * 0.8, assuming time_step = 0.1 for now
            }

            mpc_params = self.load_mpc_params(veh_params)

            steer_command, acc, steer_feedforward = self.calc_mpc(trajref, delta_x, veh_pose, ref_pose, mpc_params, int(self.l_idx), veh_params)
            # print("first output : ", steer_command)
            steer_cmd = steer_feedforward + steer_command # steer cmd is degree unit
            steer_cmd = self.limit_steer_by_angular_vel(steer_cmd, CS.actuators.steer/self.steer_ratio/180*np.pi, veh_params['max_angular_vel'], 0.1)
            steer = self.limit_steer_angle(steer_cmd, veh_params['max_steer_angle'])
            print("wheel : ",steer)
            steer = steer * self.steer_ratio
            print("steering : " ,steer)
            print("==================================")

            # wheel_angle, lah_pt = self.purepursuit.run(
            #     CS.vEgo, self.local_path[int(self.l_idx):], (CS.position.x, CS.position.y), CS.yawRate)
            
            # wheel_angle = self.stanley.run(
            #     CS.vEgo, self.local_path[int(self.l_idx):], (CS.position.x, CS.position.y), CS.yawRate)
            
            # steer = wheel_angle*self.steer_ratio
            
            # lah_viz = LookAheadViz(lah_pt)

            # self.pub_lah.publish(lah_viz)
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
