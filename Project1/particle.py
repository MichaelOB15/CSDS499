
import numpy as np
from math import sin, cos, sqrt, pi
import math


class Particle:
    def __init__(self, config, pose = [], occupancy_map = None, occupancy_weight_map = None):  # get rid of occupancy weight map once this compiles
        """Initialize the particle at the center of its internal map. Use pg 478 as a reference for an overview of the full algorithm"""

        self.config = config

        # initial map size without any resizes
        row = config.initial_occupancy_map_size[0]
        col = config.initial_occupancy_map_size[1]

        # error matrix-- this might be a weird place to put this but it's the same between all robots
        self.a=config.alpha

        # initial condition -> -1 is not yet identified, 1 is an object, 0 is open
        if occupancy_map is None:
            self.map=np.ones((row,col)) - config.initial_weight
            # self.occupancy_weight_map = np.ones((row,col)) - config.initial_weight
        else:
            self.map = occupancy_map
            # self.occupancy_weight_map = occupancy_weight_map

        if len(pose) == 0:
            self.pose=np.array([row/2, col/2, 0])  # y,x,theta
        else:
            self.pose = pose

        self.weight = 1  # weight of the particle, not the map
        self.measurements = []

    def __str__(self) -> str:
        # TODO maybe map?
        return f'pose = {self.pose}, weight = {self.weight}, measurements = {self.measurements}'

    def set_measurement(self, measurement):
        self.measurements = measurement

    # def set_pose(self, pose):
        # self.pose = pose

    def get_pose(self):
        return self.pose

    def get_map(self):
        return self.map

    def get_weight(self):
        return self.weight

    def sample_motion_model_velocity(self, u0, stepsize):
        '''Move the location of the robot with trajectory error'''

        a = self.a

        v = u0[0]
        w = u0[1]

        variance = np.zeros(3)
        variance[0]=a[0]*abs(v)+a[1]*abs(w)
        variance[1]=a[2]*abs(v)+a[3]*abs(w)
        variance[2]=a[4]*abs(v)+a[5]*abs(w)

        offset = 1e-18

        # avoids any divide by zero errors
        variance = variance + offset

        # sample normal distribution:
        mu = 0
        sigma = np.zeros(3)
        for i in range(len(variance)):
            sigma[i]=sqrt(variance[i])
        epsilon=np.random.normal(mu,sigma)

        # add error to motion command
        u = u0+epsilon[0:1]

        # reset motion, this time with error
        v = u[0]
        w = u[1]
        theta=self.pose[2]

        # apply motion
        x_update=-v/w*sin(theta)+v/w*sin(theta+w*stepsize)
        y_update=v/w*cos(theta)-v/w*cos(theta+w*stepsize)
        theta_update=(w+epsilon[2])*stepsize

        self.pose = self.pose+np.array([y_update, x_update, theta_update])
        # TODO might need to add wall collision

    def inverse_range_sensor_model(self, m_i, sensor):
        """Implements the inverse measurement model seen on pg 288"""

        rmax = self.config.rmax
        # alpha = 1

        l_occ = self.config.l_occ
        l_free = self.config.l_free
        lo = self.config.l_o

        x_i = m_i[1]
        y_i = m_i[0]

        r = sqrt((x_i - self.pose[1])**2+(y_i - self.pose[0])**2)

        z_t_k = [self.measurements[0][sensor], self.measurements[1][sensor]]

        rmax_offset = 1

        if (r > (rmax-rmax_offset)):
            return lo
        elif r < z_t_k[0]:
            return l_free
        else:
            return l_occ

    def likelihood_field_range_finder_model(self):
        """This method is heavily modified but implements the algorithm seen on pg 172"""

        q = 1

        zmax = self.config.zmax
        zhit = self.config.zhit
        zrandom = self.config.zrandom
        sigma_hit = self.config.sigma_hit

        rout=np.zeros(self.config.num_sensors)+self.config.rmax

        for s in range(self.config.num_sensors):

            # efficient code to get exactly the perceptual field of a sensor
            perceptual_field = self.perceptual_field(s)

            # go through every cell of the perceptual field
            index=len(perceptual_field)

            for i in range(index):
                # print(i)
                row=perceptual_field[i][1]
                col=perceptual_field[i][0]

                # find the corresponding closest radius for the sensor
                if self.map[row][col]== 1:
                    r=sqrt((self.pose[0]-row)**2+(self.pose[1]-col)**2)

                    # the smallest value is the closest radius
                    if r<rout[s]:
                        rout[s]=r

            difference = abs(self.measurements[0][s]-rout[s])

            # error between the recieved sensor value and the expected one placed on normal distribution
            norm = np.random.normal(difference, sigma_hit)
            q = q*(zhit*norm + zrandom/zmax)

        assert q > 0
        self.weight = q

    def update_occupancy_grid(self):
        '''update the map based on measurement data'''

        lo = self.config.l_o

        for s in range(self.config.num_sensors):
            perceptual_field = self.perceptual_field(s)

            # go through every cell of the perceptual field
            index=len(perceptual_field)
            for i in range(index):
                row=perceptual_field[i][1]
                col=perceptual_field[i][0]

                self.map[row,col] = self.map[row,col] + self.inverse_range_sensor_model([row,col], s) - lo

        self.resize()

    def perceptual_field(self, s):
        """return the cells that are imaged in a sensor reading"""

        num_sensors = self.config.num_sensors
        theta = s*2*pi/num_sensors
        dtheta=self.config.dtheta
        beam_width = self.config.beam_width
        spread = beam_width*pi/180

        # rmax= self.config.rmax
        dr= self.config.dr

        r_steps=int((self.measurements[0][s]+2)/dr)
        theta_steps=int(spread/dtheta)

        distinct_pairs = []

        for i in range(theta_steps):
            for j in range(r_steps):
                temp_angle=self.pose[2]+theta-spread/2+i*dtheta
                temp_r=j*dr

                x=int(self.pose[1]+temp_r*cos(temp_angle))
                y=int(self.pose[0]+temp_r*sin(temp_angle))

                if [x,y] not in distinct_pairs:
                    distinct_pairs.append([x,y])
                else:
                    pass

        return distinct_pairs

    def resize(self):
        '''Determines whether the robot's internal map is  at risk of being too small and resizes it accordingly'''

        n_row=np.shape(self.map)[0]
        n_col=np.shape(self.map)[1]

        # I need to go around every edge (with a cushion) and see if I have room to work
        cushion=self.config.cushion  # if these two are added to .yaml file then setpose method also could use these
        resize_magnitude=200

        x_pos = self.pose[0]
        y_pos = self.pose[1]

        # unmapped regions will hold a value of -1

        # row underflow (x axis)
        if (x_pos <= cushion):
            newmap=np.zeros((n_row,resize_magnitude))-1
            # new_weight_map=np.zeros((n_row,resize_magnitude))+.5
            self.map=np.concatenate([newmap, self.map],axis=1)
            # self.occupancy_weight_map=np.concatenate([new_weight_map, self.occupancy_weight_map],axis=1)
            self.pose[0] += cushion
            n_col += 200

        # row overflow (x axis)
        if (x_pos >= (n_col - cushion)):
            newmap=np.zeros((n_row,resize_magnitude))-1
            # new_weight_map=np.zeros((n_row,resize_magnitude))+.5
            self.map=np.concatenate([self.map, newmap],axis=1)
            # self.occupancy_weight_map=np.concatenate([self.map, new_weight_map],axis=1)
            n_col += 200

        # column underflow (y axis)
        if (y_pos <= cushion):
            newmap=np.zeros((resize_magnitude,n_col))-1
            # new_weight_map=np.zeros((resize_magnitude,n_col))+.5
            self.map=np.concatenate([newmap, self.map],axis=0)
            # self.occupancy_weight_map=np.concatenate([new_weight_map, self.map],axis=0)
            self.pose[1] += cushion

        # column overflow (x axis)
        if (y_pos >= (n_row - cushion)):
            newmap=np.zeros((resize_magnitude,n_col))-1
            # new_weight_map=np.zeros((resize_magnitude,n_col))+.5
            self.map=np.concatenate([self.map, newmap],axis=0)
            # self.occupancy_weight_map=np.concatenate([self.map, new_weight_map],axis=0)

        return self.map
