import numpy as np
from math import sin, cos, sqrt, pi

class Particle:
    def __init__(self, position = None, map = None):
        """Initialize the particle at the center of its internal map."""

        # TODO Add these variables to the config

        #initial map size without any resizes
        row,col=300,300

        #error matrix-- this might be a weird place to put this but it's the same between all robots
        self.a=np.array([0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001])

        #initial condition - [-1 is not yet identifie, 1 is an object, 0 is open]
        if map == None:
            self.map=np.zeros((row,col))-1

        self.pose=np.array(position) #y,x,theta

        self.weight=None
        self.measurements=None

    #use pg 478 as a reference for an overview of the full algorithm

    def setmeasurements(self, measurement):
        self.measurements = measurement

    def setpose(self,pose):
        self.pose = pose

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

    def measurement_model_map(self,z): #z in this method comes from measurement.py and is sent in from the main.py script
        '''Set the weight of the particle'''
        pass

    def update_occupancy_grid(self):
        '''update the map based on measurement data'''

        n_row=np.shape(self.map)[0]
        n_col=np.shape(self.map)[1]

        new_map = np.zeros((n_row,n_col))

        time_steps = len(self.measurements)

        prand = .5
        zmax = 30
        sigma = .5
        phit = .5

        # repeat until convergence

        for t in range(time_steps):
            # For time t get all D_t and K_t
            D_t = sorted(self.measurements)
            K_t = len(self.measurements[time_steps])

            e_t_star = prand * sqrt(2*pi*sigma**2)/zmax

            e_t = []
            e_t.append((1-phit)**2 * math.exp(-()/(sigma**2 * 2)))
        
        for x in range(n_col):
            for y in range(n_row):
                # check both the occupied and unoccupied of each cell

                # whichever is higher set that as the new value


        # get measurement
        # upda
        
        pass
    
    def update_particle_map(self, position, value):
        self.map[position[0], position[1]] = value

    
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