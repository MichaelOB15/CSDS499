
import numpy as np
from math import sin, cos, sqrt, pi
import math

class Particle:
    def __init__(self, pose = None, map = None):
        """Initialize the particle at the center of its internal map."""

        # TODO Add these variables to the config

        #initial map size without any resizes
        row,col=300,300

        #error matrix-- this might be a weird place to put this but it's the same between all robots
        self.a=np.array([0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001])

        #initial condition -> -1 is not yet identified, 1 is an object, 0 is open
        if map == None:
            self.map=np.zeros((row,col))-1

        if pose == None:
            self.pose=np.array([row/2, col/2, 0]) #y,x,theta
        else:
            self.setpose(pose)

        self.weight=None
        self.measurements= []
        
    #use pg 478 as a reference for an overview of the full algorithm

    def addmeasurements(self, measurement):
        self.measurements.append(measurement)

    def setpose(self,pose):
        #pose has the potential to leave the map
        map=self.map

        #map will be expanded by the difference between the default pose plus a cushion
        cushion=50 #note that this method means repeated setpose() calls will repeatedly upsize the map
        
        row_error=self.pose[0]-pose[0] +cushion #instead of having the cushion be the size of the map use values from .yaml file
        col_error=self.pose[1]-pose[1] +cushion
        
        #value needs to be an integer greater than zero
        if row_error<1:
            row_error=1
        if col_error<1:
            col_error=1
            
        n_col=np.shape(map)[1]
        
        #row overflow code seen in resize()
        newmap=np.zeros((int(row_error),n_col))-1
        map=np.concatenate([map,newmap],axis=0)

        n_row=np.shape(map)[0]

        #column overflow code seen in resize()
        newmap=np.zeros((n_row,int(col_error)))-1
        map=np.concatenate([map,newmap],axis=1)

        self.map=map
        self.pose=pose

    def getpose(self):
        return self.pose

    def getmap(self):
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

    def get_weight(self):
        return self.weight

    def inverse_range_sensor_model(self, m_i):

        # CHANGE LOC LFREE AND LO

        zmax = 30
        alpha = 1

        l_occ = np.log(0.65/0.35)
        l_free = np.log(0.35/0.65)
        lo = np.log(0.4/.0,6)

        beam_width = 15 #15 degree beam width, this should probably go in the config file
        beta = beam_width*pi/180

        x_i = m_i[0] + .5
        y_i = m_i[1] + .5

        r = sqrt((x_i - self.pose[0])**2+(y_i - self.pose[1])**2)
        phi = math.atan2((y_i - self.pose[1]),(x_i - self.pose[0])) - self.pose[2]

        k = 0
        min_val = 2*math.pi
        for j in range(len(self.measurements[0])):
            new_val = math.abs(phi - self.measurements[-1][1][j])
            if min_val > new_val:
                min_val = new_val
                k = j

        z_t_k = [self.measurements[-1][0][k],self.measurements[-1][1][k]]
            
        if (r > math.min(zmax, z_t_k[0] + alpha/2)) or (math.abs(phi - z_t_k[1]) >  beta/2):
            return lo
        if z_t_k[0] < zmax and math.abs(r - z_t_k[0]) < alpha/2:
            return l_occ
        if r <= z_t_k[0]:
            return l_free

    def likelihood_field_range_finder_model(self):
        q = 1
        zmax = 30
        zhit = .5
        zrandom = .5
        sigma_hit = .5

        n_row=np.shape(self.map)[0]
        n_col=np.shape(self.map)[1]

        for k in range(len([self.measurements[0]])):
            z_t_k = [self.measurements[-1][0][k],self.measurements[-1][1][k]]
            if z_t_k != zmax:
                x_k_sensor = self.measurements[-1][0][k]*cos(self.measurements[-1][1][k]) + self.pose[0]
                y_k_sensor = self.measurements[-1][0][k]*sin(self.measurements[-1][1][k]) + self.pose[1]
                x_z_k_t = self.pose[0] + x_k_sensor * cos(self.pos[2]) - y_k_sensor * sin(self.pos[2]) + z_t_k[0] * cos(self.pose[2] + self.measurements[-1][1][k])
                y_z_k_t = self.pose[1] + y_k_sensor * cos(self.pos[2]) - x_k_sensor * sin(self.pos[2]) + z_t_k[0] * sin(self.pose[2] + self.measurements[-1][1][k])

                min_dist = zmax + 1

                for x_prime in range(n_col):
                    for y_prime in range(n_row):
                        if self.map[x,y] == 1:
                            dist = sqrt((x_z_k_t- x_prime)**2+(y_z_k_t-y_prime)**2)
                            if dist < min_dist:
                                min_dist = dist
                
                q = q * (zhit * self.prob(dist, sigma_hit)+ zrandom/zmax)
        
        return q

    def prob(self, a, b):
        return np.random.normal(a, b)


    def update_occupancy_grid(self, prev_weights):
        '''update the map based on measurement data'''

        lo = np.log(0.4/.0,6)

        n_row=np.shape(self.map)[0]
        n_col=np.shape(self.map)[1]

        perceptual_field = []
        for sensor in range(len(self.measurements[0])):
            perceptual_field.append(self.perceptual_field(sensor))
                
            for x in range(n_col):
                for y in range(n_row):
                    if [x,y] in perceptual_field:
                        prev_weights[x,y] = prev_weights[x,y] + self.inverse_range_sensor_model([x,y]) - lo
                    else:
                        pass

        self.resize()
            
        return prev_weights

    # update map from weights
    def update_map_from_weights(self, weights):
        n_row=np.shape(weights)[0]
        n_col=np.shape(weights)[1]

        for x in range(n_col):
            for y in range(n_row):
                if weights[x,y] > .5:
                    self.map[x,y] = 1
                else:
                    self.map[x,y] = 0


    def perceptual_field(self, sensor):
        rmax=30
        dr=0.3
        dtheta=2*pi/360 #every degree -> I think the robot is set up for 15 degrees

        num_sensors = 6

        beam_width = 15 #15 degree beam width, this should probably go in the config file
        spread = beam_width*pi/180

        theta = sensor*2*pi/num_sensors

        r_steps=int(rmax/dr)
        theta_steps=int(spread/dtheta)

        distinct_pairs = []

        for i in range(theta_steps):
            for j in range(r_steps):
                temp_angle=pose[2]+theta-spread/2+i*dtheta
                temp_r=j*dr

                x=int(pose[1]+temp_r*cos(temp_angle))
                y=int(pose[0]+temp_r*sin(temp_angle))

                if [x,y] not in distinct_pairs:
                    distinct_pairs.append([x,y])
                else:
                    pass
        
        return distinct_pairs

    
    #####particle takes a measurement with all sensors and back-calculates likelihood of position (measurement model map)
    #here is where we write the sensor model -> use several spaced out ultrasound sensors, and these sensors are described as:
    # the correct algorithm for range finder sensor modelling is on book pg 172 (this is too simple)
    # localization using an ultrasound sensor is on pg 288 -> inverse problem (I think this is the right one)
    # I think we want a combo of pg 301 algorithm plus the dummy pg 172 one for just the mapping... (this is wrong but I dont want to lose page)
    #book has their robot cone opening 15 degrees
    #based on pg 303 it's not unusual to have a robot with evenly spaced sensors on all sides

    def resize(self):
        '''Determines whether the robot's internal map is  at risk of being too small and resizes it accordingly'''
        
        n_row=np.shape(self.map)[0]
        n_col=np.shape(self.map)[1]

        #I need to go around every edge (with a cushion) and see if I have room to work
        cushion=50 ########################################if these two are added to .yaml file then setpose method also could use these 
        resize_magnitude=200

        x_pos = self.pose[0]
        y_pos = self.pose[1]

        #unmapped regions will hold a value of -1

        # row underflow (x axis)
        if (x_pos <= cushion):
            newmap=np.zeros((n_row,resize_magnitude))-1
            self.map=np.concatenate([newmap, self.map],axis=1)
            self.pose[0] += cushion
            n_col += 200

        # row overflow (x axis)
        if (x_pos >= (n_col - cushion)):
            newmap=np.zeros((n_row,resize_magnitude))-1
            self.map=np.concatenate([self.map, newmap],axis=1)
            n_col += 200

        # column underflow (y axis)
        if (y_pos <= cushion): 
            newmap=np.zeros((resize_magnitude,n_col))-1
            self.map=np.concatenate([newmap, self.map],axis=0)
            self.pose[1] += cushion

        # column overflow (x axis)
        if (y_pos >= (n_row - cushion)):
            newmap=np.zeros((resize_magnitude,n_col))-1
            self.map=np.concatenate([self.map, newmap],axis=0)

        return self.map
