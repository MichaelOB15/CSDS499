
import numpy as np
from math import sin, cos, sqrt, pi
import math

class Particle:
    def __init__(self, config, pose = [], occupancy_map = None, occupancy_weight_map = None): #get rid of occupancy weight map once this compiles
        """Initialize the particle at the center of its internal map. Use pg 478 as a reference for an overview of the full algorithm"""

        self.config = config

        #initial map size without any resizes
        row = config.initial_occupancy_map_size[0]
        col = config.initial_occupancy_map_size[1]

        #error matrix-- this might be a weird place to put this but it's the same between all robots
        self.a=config.alpha

        #initial condition -> -1 is not yet identified, 1 is an object, 0 is open
        if occupancy_map == None:
            self.map=np.ones((row,col)) - config.initial_weight
            #self.occupancy_weight_map = np.ones((row,col)) - config.initial_weight
        else:
            self.map = occupancy_map
            #self.occupancy_weight_map = occupancy_weight_map

        if len(pose) == 0:
            self.pose=np.array([row/2, col/2, 0]) #y,x,theta
        else:
            self.set_pose(pose)

        self.weight = 1 #weight of the particle, not the map
        self.measurements = []

    def set_measurement(self, measurement):
        self.measurements = measurement

    def set_pose(self,pose):
        self.pose = pose

    def get_pose(self):
        return self.pose

    def get_map(self):
        return self.map

    def sample_motion_model_velocity(self,u0,stepsize):
        '''Move the location of the robot with trajectory error'''

        a=self.a

        v=u0[0]
        w=u0[1]

        variance = np.zeros(3)
        variance[0]=a[0]*abs(v)+a[1]*abs(w)
        variance[1]=a[2]*abs(v)+a[3]*abs(w)
        variance[2]=a[4]*abs(v)+a[5]*abs(w)

        offset=1e-18

        #avoids any divide by zero errors
        variance = variance + offset

        #sample normal distribution:
        mu=0
        sigma=np.zeros(3)
        for i in range(len(variance)):
            sigma[i]=sqrt(variance[i])
        epsilon=np.random.normal(mu,sigma)

        #add error to motion command
        u=u0+epsilon[0:1]

        #reset motion, this time with error
        v=u[0]
        w=u[1]
        theta=self.pose[2]

        #apply motion
        x_update=-v/w*sin(theta)+v/w*sin(theta+w*stepsize)
        y_update=v/w*cos(theta)-v/w*cos(theta+w*stepsize)
        theta_update=(w+epsilon[2])*stepsize

        self.pose=self.pose+np.array([y_update,x_update,theta_update])
        # TODO might need to add wall collision

    def get_weight(self):
        return self.weight

    def inverse_range_sensor_model(self, m_i):
        """Implements the inverse measurement model seen on pg 288"""

        zmax = self.config.zmax
        alpha = 1 #this parameter is hard-coded -> cell size is always 1

        l_occ = self.config.l_occ # these values need to be tuned!
        l_free = self.config.l_free
        lo = self.config.l_o

        beam_width = self.config.beam_width
        beta = beam_width*pi/180

        x_i = m_i[0] + .5 #we maybe should get rid of this-- it doesnt really matter and its just weird
        y_i = m_i[1] + .5

        r = sqrt((x_i - self.pose[0])**2+(y_i - self.pose[1])**2)
        phi = math.atan2((y_i - self.pose[1]),(x_i - self.pose[0])) - self.pose[2]

        k = 0 #I need to think about this line again this doesn't quite seem right
        min_val = 2*math.pi
        for j in range(len(self.measurements[0])):
            new_val = abs(phi - self.measurements[1][j])
            if min_val > new_val:
                min_val = new_val
                k = j

        z_t_k = [self.measurements[0][k],self.measurements[1][k]]
            
        if (r > min(zmax, z_t_k[0] + alpha/2)) or (abs(phi - z_t_k[1]) >  beta/2):
            return lo
        if z_t_k[0] < zmax and math.abs(r - z_t_k[0]) < alpha/2:
            return l_occ
        if r <= z_t_k[0]:
            return l_free

    def likelihood_field_range_finder_model(self):
        """This method is heavily modified but implements the algorithm seen on pg 172"""

        q = 1

        zmax = self.config.zmax
        zhit = self.config.zhit
        zrandom = self.config.zrandom
        sigma_hit = self.config.sigma_hit #get these into a config and tune them :(   
        
        rout=np.zeros(self.config.num_sensors)+self.config.rmax

        for s in range(self.config.num_sensors):

            #efficient code to get exactly the perceptual field of a sensor
            perceptual_field = self.perceptual_field(s)
            print(perceptual_field[0][0])

            #go through every cell of the perceptual field
            index=len(perceptual_field[0])
            for i in range(index):
                row=perceptual_field[0][i]
                col=perceptual_field[1][i]

                #find the corresponding closest radius for the sensor
                if self.map[row,col]==1:
                    r=sqrt((self.pose[0]-row).pow(2)+(self.pose[1]-col).pow(2))

                    #the smallest value is the closest radius
                    if r<rout[s]:
                        rout[s]=r

            difference=self.measurements[0][s]-rout[s]

            #error between the recieved sensor value and the expected one placed on normal distribution 
            q=q*(zhit*np.random.normal(difference,sigma_hit)+zrandom/zmax)

        self.weight = q


    def update_occupancy_grid(self):
        '''update the map based on measurement data'''

        lo = self.config.l_o

        for s in range(self.config.num_sensors):
            perceptual_field = self.perceptual_field(s) 

            index=len(perceptual_field)
            for i in range(index):
                row=perceptual_field[0,i]
                col=perceptual_field[1,i]

                self.occupancy_weight_map[row,col] = self.occupancy_weight_map[row,col] + self.inverse_range_sensor_model([row,col]) - lo

                #I kinda want the actual probability as an output and then we can have probability fields in the output image
                #why do we have two maps? the only one we need is the probability one??
                '''
                if self.occupancy_weight_map[row,col] > 0.5:
                    self.map[row,col] = 1
                else:
                    self.map[row,col] = 0
                '''

        self.resize()

    def perceptual_field(self, s):
        """return the cells that are imaged in a sensor reading"""

        num_sensors = self.config.num_sensors
        theta = s*2*pi/num_sensors
        dtheta=self.config.dtheta
        beam_width = self.config.beam_width
        spread = beam_width*pi/180

        rmax= self.config.rmax
        dr= self.config.dr

        r_steps=int(rmax/dr)
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

        #I need to go around every edge (with a cushion) and see if I have room to work
        cushion=self.config.cushion ########################################if these two are added to .yaml file then setpose method also could use these 
        resize_magnitude=200

        x_pos = self.pose[0]
        y_pos = self.pose[1]

        #unmapped regions will hold a value of -1

        # row underflow (x axis)
        if (x_pos <= cushion):
            newmap=np.zeros((n_row,resize_magnitude))-1
            new_weight_map=np.zeros((n_row,resize_magnitude))+.5
            self.map=np.concatenate([newmap, self.map],axis=1)
            self.occupancy_weight_map=np.concatenate([new_weight_map, self.occupancy_weight_map],axis=1)
            self.pose[0] += cushion
            n_col += 200

        # row overflow (x axis)
        if (x_pos >= (n_col - cushion)):
            newmap=np.zeros((n_row,resize_magnitude))-1
            new_weight_map=np.zeros((n_row,resize_magnitude))+.5
            self.map=np.concatenate([self.map, newmap],axis=1)
            self.occupancy_weight_map=np.concatenate([self.map, new_weight_map],axis=1)
            n_col += 200

        # column underflow (y axis)
        if (y_pos <= cushion): 
            newmap=np.zeros((resize_magnitude,n_col))-1
            new_weight_map=np.zeros((resize_magnitude,n_col))+.5
            self.map=np.concatenate([newmap, self.map],axis=0)
            self.occupancy_weight_map=np.concatenate([new_weight_map, self.map],axis=0)
            self.pose[1] += cushion

        # column overflow (x axis)
        if (y_pos >= (n_row - cushion)):
            newmap=np.zeros((resize_magnitude,n_col))-1
            new_weight_map=np.zeros((resize_magnitude,n_col))+.5
            self.map=np.concatenate([self.map, newmap],axis=0)
            self.occupancy_weight_map=np.concatenate([self.map, new_weight_map],axis=0)

        return self.map
