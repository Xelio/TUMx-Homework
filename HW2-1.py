from plot import plot

class UserCode:
    def __init__(self):
        # initialize data you want to store in this object between calls to the measurement_callback() method
        self.last_yaw = 0
        self.max_roll = 0
        self.max_pitch = 0
        self.max_v_yaw = 0
        
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        # add your plot commands here
        self.max_roll = max(self.max_roll, abs(navdata.rotX));
        self.max_pitch = max(self.max_pitch, abs(navdata.rotY));
        self.max_v_yaw = max(self.max_v_yaw, abs((navdata.rotZ - self.last_yaw)/dt));
        self.last_yaw = navdata.rotZ;
        
        plot("max_roll", self.max_roll);
        plot("max_pitch", self.max_pitch);
        plot("max_v_yaw", self.max_v_yaw);
