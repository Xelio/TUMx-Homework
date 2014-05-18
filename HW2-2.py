import numpy as np
from plot import plot_trajectory
from math import sin, cos

class UserCode:
    def __init__(self):
        self.position = np.array([[0], [0]])
        
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        
        # TODO: update self.position by integrating measurements contained in navdata
        R = np.array([[cos(navdata.rotZ),-1*sin(navdata.rotZ),self.position[0]],
                      [sin(navdata.rotZ),cos(navdata.rotZ),self.position[1]],
                      [0,0,1]]);
        
        x = navdata.vx * dt;
        y = navdata.vy * dt;
        P_global = np.dot(R, np.array([[x],[y],[1]]));
        self.position = np.array([[P_global[0]],[P_global[1]]]);
        plot_trajectory("odometry", self.position)
