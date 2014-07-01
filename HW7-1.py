import math
import numpy as np
from plot import plot, plot_trajectory, plot_covariance_2d
from math import cos, sin

class UserCode:
    def __init__(self):
        # xy control gains
        self.Kp_xy = 1.9 # xy proportional
        self.Kd_xy = 0.1 # xy differential
        self.Kh_xy = 0.1 # xy heading correction
        
        # yaw control gains
        self.Kp_yaw  = 0.1 # yaw proportional
        self.Kd_yaw  = 0.01 # yaw differential
		
        # state vector [x, y, yaw] in world coordinates
        self.x = np.zeros((3,1))
		
		# 3x3 state covariance matrix
        self.sigma = 0.01 * np.identity(3)
	    
        # target marker
        self.target_marker_index = 0
        self.distance_marker_reached = 1.2
		
		#process noise
        pos_noise_std = 0.009
        yaw_noise_std = 0.009
        self.Q = np.array([
            [pos_noise_std*pos_noise_std,0,0],
            [0,pos_noise_std*pos_noise_std,0],
            [0,0,yaw_noise_std*yaw_noise_std]
        ])
        
        #measurement noise
        z_pos_noise_std = 0.00005
        z_yaw_noise_std = 0.00005
        self.R = np.array([
            [z_pos_noise_std*z_pos_noise_std,0,0],
            [0,z_pos_noise_std*z_pos_noise_std,0],
            [0,0,z_yaw_noise_std*z_yaw_noise_std]
        ])
        
    def get_markers(self):
        '''
        place up to 30 markers in the world
        '''
        markers = [
             [0, 0], # marker at world position x = 0, y = 0
             [3.8, 0.1],  # marker at world position x = 2, y = 0
			 [3.5, 1.8],
			 [1.6, 3.4],
			 [4.3, 3.3],
			 [6.8, 4.8],
			 [4.1, 5.4],
			 [4.1, 8.2],
			 [7.5, 7.9],
			 [9.3, 8.7],
			 [9.1, 9.5],
			 [9.2, 12.2],
			 [7.4, 11.0],
			 [5.0, 11.0]
        ]
        
        #TODO: Add your markers where needed
       
        return markers
        
    def rotation(self, yaw):
        '''
        create 2D rotation matrix from given angle
        '''
        s_yaw = math.sin(yaw)
        c_yaw = math.cos(yaw)
                
        return np.array([
            [c_yaw, -s_yaw], 
            [s_yaw,  c_yaw]
        ])
        
    def normalizeYaw(self, y):
        '''
        normalizes the given angle to the interval [-pi, +pi]
        '''
        while(y > math.pi):
            y -= 2 * math.pi
        while(y < -math.pi):
            y += 2 * math.pi
        return y

        
    def predictState(self, dt, x, u_linear_velocity, u_yaw_velocity):
        '''
        predicts the next state using the current state and 
        the control inputs local linear velocity and yaw velocity
        '''
        x_p = np.zeros((3, 1))
        x_p[0:2] = x[0:2] + dt * np.dot(self.rotation(x[2]), u_linear_velocity)
        x_p[2]   = x[2]   + dt * u_yaw_velocity
        x_p[2]   = self.normalizeYaw(x_p[2])
        
        return x_p
        
    def calculatePredictStateJacobian(self, dt, x, u_linear_velocity, u_yaw_velocity):
        '''
        calculates the 3x3 Jacobian matrix for the predictState(...) function
        '''
        s_yaw = math.sin(x[2])
        c_yaw = math.cos(x[2])
        
        dRotation_dYaw = np.array([
            [-s_yaw, -c_yaw],
            [ c_yaw, -s_yaw]
        ])
        F = np.identity(3)
        F[0:2, 2] = dt * np.dot(dRotation_dYaw, u_linear_velocity)
        
        return F
        
    def predictCovariance(self, sigma, F, Q):
        '''
        predicts the next state covariance given the current covariance, 
        the Jacobian of the predictState(...) function F and the process noise Q
        '''
        return np.dot(F, np.dot(sigma, F.T)) + Q

    def compute_heading_correction(self, position, velocity, position_desired):
        vector_position_to_desire = position_desired - position
        b = velocity
        a = vector_position_to_desire
        return -1 * np.dot((np.identity(2) - np.dot(a / np.dot(a.T, a)[0], a.T)), b)
        
    def compute_control_command(self, t, dt, position, velocity, position_desired, velocity_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param position: current quadrotor position
        :param velocity:current quadrotor velocity
        :param position_desired: State - desired quadrotor position
        :param position_desired: State - desired quadrotor velocity
        :return - xy velocity control signal represented as 2x1 numpy array
        '''
        
        up = self.Kp_xy * (position_desired - position)
        ud = self.Kd_xy * (velocity_desired - velocity)
        uh = self.Kh_xy * self.compute_heading_correction(position, velocity, position_desired)
        
        u = up + ud + uh
        
        return u
        
    def compute_yaw_control_command(self, t, dt, yaw, yaw_velocity, yaw_desired, yaw_velocity_desired):
        up = self.Kp_xy * (yaw_desired - yaw)
        ud = self.Kd_xy * (yaw_velocity_desired - yaw_velocity)
        
        u = up + ud
        
        return u

    def norm(self, vector):
        return math.sqrt(vector[0]**2 + vector[1]**2)
        
    def update_target_marker_index(self, markers, current_target_marker_index, position):
        '''
        :param markers: list of markers
        :param current_target_marker_index: index of current target marker
        :param position: current quadrotor position
        :return - index of new target marker
        '''
        current_target_marker = markers[current_target_marker_index]
        diff = np.array(current_target_marker) - position
        dist = self.norm(diff)
        
        if dist < self.distance_marker_reached:
            return min(current_target_marker_index + 1, len(markers) - 1)
        else:
            return current_target_marker_index
            
    def compute_desired_velocity(self, markers, current_target_marker_index, position, velocity):
        '''
        :param markers: list of markers
        :param current_target_marker_index: index of current target marker
        :param position: current quadrotor position
        :param velocity: current quadrotor velocity
        :return - desired xy velocity represented as 2x1 numpy array
        '''
        if current_target_marker_index == len(markers) - 1:
            return np.array([[0],[0]])
        
        next_target_marker_index = current_target_marker_index + 1
        
        current_target_marker = np.array([markers[current_target_marker_index]]).transpose()
        next_target_marker = np.array([markers[next_target_marker_index]]).transpose()
        
        vector_current_to_next = next_target_marker - current_target_marker
        vector_position_to_current = current_target_marker - position
        
        dist_current_to_next = self.norm(vector_current_to_next)
        dist_position_to_current = self.norm(vector_position_to_current)
        
        if dist_position_to_current > self.distance_marker_reached * 5:
            return vector_position_to_current / 3
            
        if dist_position_to_current > self.distance_marker_reached * 3:
            return vector_position_to_current / 5
        
        return -1 * vector_position_to_current + vector_current_to_next
    
    def state_callback(self, t, dt, linear_velocity, yaw_velocity):
        '''
        called when a new odometry measurement arrives approx. 200Hz
    
        :param t - simulation time
        :param dt - time difference this last invocation
        :param linear_velocity - x and y velocity in local quadrotor coordinate frame (independet of roll and pitch)
        :param yaw_velocity - velocity around quadrotor z axis (independet of roll and pitch)

        :return tuple containing linear x and y velocity control commands in local quadrotor coordinate frame (independet of roll and pitch), and yaw velocity
        '''
        self.x = self.predictState(dt, self.x, linear_velocity, yaw_velocity)
		
        F = self.calculatePredictStateJacobian(dt, self.x, linear_velocity, yaw_velocity)
        self.sigma = self.predictCovariance(self.sigma, F, self.Q);
        
        position = self.x[0:2]
        rotation_matrix = self.rotation(self.x[2])
        markers = self.get_markers()
        self.target_marker_index = self.update_target_marker_index(markers, self.target_marker_index, position)
        
        desired_velocity_global = self.compute_desired_velocity(markers, self.target_marker_index, position, linear_velocity)
        desired_velocity = np.dot(rotation_matrix, desired_velocity_global)
        
        u_global = self.compute_control_command(t, dt, position, linear_velocity, np.array([markers[self.target_marker_index]]).T, desired_velocity)
        u = np.dot(rotation_matrix, u_global)
        
        u_yaw = self.compute_yaw_control_command(t, dt, self.x[2], yaw_velocity, 0, 0)
        #plot("desire x velocity", desired_velocity[0]);
        #plot("desire y velocity", desired_velocity[1]);
        plot("x command", u[0]);
        #plot("y command", u[1]);
        # plot("yaw command", u_yaw);
        
        # plot("desire x", markers[self.target_marker_index][0]);
        # plot("desire y", markers[self.target_marker_index][1]);
        self.visualizeState()
        
        return u, u_yaw


        

        
        
        
        
    def calculateKalmanGain(self, sigma_p, H, R):
        '''
        calculates the Kalman gain
        '''
        return np.dot(np.dot(sigma_p, H.T), np.linalg.inv(np.dot(H, np.dot(sigma_p, H.T)) + R))
        
    def correctState(self, K, x_predicted, z, z_predicted):
        '''
        corrects the current state prediction using Kalman gain, the measurement and the predicted measurement
        
        :param K - Kalman gain
        :param x_predicted - predicted state 3x1 vector
        :param z - measurement 3x1 vector
        :param z_predicted - predicted measurement 3x1 vector
        :return corrected state as 3x1 vector
        '''
        z_diff = (z - z_predicted)
        z_diff[2] = self.normalizeYaw(z_diff[2])
            
        return x_predicted + np.dot(K, z_diff)
        
    def correctCovariance(self, sigma_p, K, H):
        '''
        corrects the sate covariance matrix using Kalman gain and the Jacobian matrix of the predictMeasurement(...) function
        '''
        return np.dot(np.identity(3) - np.dot(K, H), sigma_p)
        
    def calculatePredictMeasurementJacobian(self, x, marker_position_world, marker_yaw_world):
        '''
        calculates the 3x3 Jacobian matrix of the predictMeasurement(...) function using the current state and 
        the marker position and orientation in world coordinates
        
        :param x - current state 3x1 vector
        :param marker_position_world - x and y position of the marker in world coordinates 2x1 vector
        :param marker_yaw_world - orientation of the marker in world coordinates
        :return - 3x3 Jacobian matrix of the predictMeasurement(...) function
        '''
        
        # TODO: implement computation of H
        x_x = x[0]
        y_x = x[1]
        psi = x[2]
        x_m = marker_position_world[0]
        y_m = marker_position_world[1]
        
        return np.array([
            [-cos(psi), -sin(psi), -(x_m - x_x)*sin(psi) + (y_m - y_x) * cos(psi)],
            [sin(psi) , -cos(psi), -(x_m - x_x)*cos(psi) - (y_m - y_x) * sin(psi)],
            [0,0,-1]
        ])
        
    def predictMeasurement(self, x, marker_position_world, marker_yaw_world):
        '''
        predicts a marker measurement given the current state and the marker position and orientation in world coordinates 
        '''
        z_predicted = Pose2D(self.rotation(x[2]), x[0:2]).inv() * Pose2D(self.rotation(marker_yaw_world), marker_position_world);
        
        return np.array([[z_predicted.translation[0], z_predicted.translation[1], z_predicted.yaw()]]).T
        

    def measurement_callback(self, marker_position_world, marker_yaw_world, marker_position_relative, marker_yaw_relative):
        '''
        called when a new marker measurement arrives max 30Hz, marker measurements are only available if the quadrotor is
        sufficiently close to a marker
            
        :param marker_position_world - x and y position of the marker in world coordinates 2x1 vector
        :param marker_yaw_world - orientation of the marker in world coordinates
        :param marker_position_relative - x and y position of the marker relative to the quadrotor 2x1 vector
        :param marker_yaw_relative - orientation of the marker relative to the quadrotor
        '''
        z = np.array([[marker_position_relative[0], marker_position_relative[1], marker_yaw_relative]]).T
        z_predicted = self.predictMeasurement(self.x, marker_position_world, marker_yaw_world)
                
        H = self.calculatePredictMeasurementJacobian(self.x, marker_position_world, marker_yaw_world)
        K = self.calculateKalmanGain(self.sigma, H, self.R)
        
        self.x = self.correctState(K, self.x, z, z_predicted)
        self.sigma = self.correctCovariance(self.sigma, K, H)
        
        self.visualizeState()
    
    def visualizeState(self):
        # visualize position state
        plot_trajectory("kalman", self.x[0:2])
        plot_covariance_2d("kalman", self.sigma[0:2,0:2])
        
        
        
class Pose2D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation
        
    def inv(self):
        '''
        inversion of this Pose2D object
        
        :return - inverse of self
        '''
        inv_rotation = self.rotation.transpose()
        inv_translation = -np.dot(inv_rotation, self.translation)
        
        return Pose2D(inv_rotation, inv_translation)
    
    def yaw(self):
        from math import atan2
        return atan2(self.rotation[1,0], self.rotation[0,0])
        
    def __mul__(self, other):
        '''
        multiplication of two Pose2D objects, e.g.:
            a = Pose2D(...) # = self
            b = Pose2D(...) # = other
            c = a * b       # = return value
        
        :param other - Pose2D right hand side
        :return - product of self and other
        '''
        return Pose2D(np.dot(self.rotation, other.rotation), np.dot(self.rotation, other.translation) + self.translation)