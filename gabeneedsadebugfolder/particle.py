import numpy as np
from math import sin, cos, sqrt

#I'm going to write a particle class -> each particle is a representation of the robot
class Particle:
    def __init__(self):
        """Initialize the particle at the center of its internal map."""

        #initial map size without any resizes
        row,col=300,300

        #initial condition
        self.map=np.zeros((row,col))-1
        self.pose=np.array([row/2,col/2,0]) #y,x,phi


    #there should be a particle.update(u) that runs the three update commands in order... implement this last

    #delete this at the very end I just need it for testing
    def setpose(self,pose):
        self.pose=pose

    '''
    things to put in config.yaml:
    initial map sizes for the particle
    a
    stepsize
    offset
    
    '''

    def sample_motion_model_velocity(self,u0,a,stepsize):
        '''Move the location of the robot with trajectory error'''

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

        self.pose=self.pose+np.array([x_update,y_update,theta_update])
        print(self.pose)















    def resize(self,map):
        '''Determines whether the robot's internal map is  at risk of being too small and resizes it accordingly'''
        
        n_row=np.shape(map)[0]
        n_col=np.shape(map)[1]

        #I need to go around every edge (with a cushion) and see if I have room to work
        cushion=40
        resize_magnitude=200

        #unmapped regions will hold a value of -1
        for a in range(n_row):
            #row overflow (x axis)
            if (not map[a,n_col-cushion]==-1):
                newmap=np.zeros((n_row,resize_magnitude))-1
                map=np.concatenate([map,newmap],axis=1)
                n_col=np.shape(map)[1]
                
            #row underflow
            if (not map[a,cushion]==-1):
                newmap=np.zeros((n_row,resize_magnitude))-1
                map=np.concatenate([newmap,map],axis=1)
                n_col=np.shape(map)[1]

                #underflows need to update pose
                self.pose[1]=self.pose[1]+resize_magnitude
                
        for a in range(n_col):
            #column overflow (y axis)
            if (not map[n_row-cushion,a]==-1):
                newmap=np.zeros((resize_magnitude,n_col))-1
                map=np.concatenate([map,newmap],axis=0)
                n_row=np.shape(map)[0]
                
            #column underflow
            if (not map[cushion,a]==-1):
                newmap=np.zeros((resize_magnitude,n_col))-1
                map=np.concatenate([newmap,map],axis=0)
                n_row=np.shape(map)[0]

                #underflows need to update pose
                self.pose[0]=self.pose[0]+resize_magnitude
        
        return map

        



        


#### class particle #####
#every particle keeps track of its own position

#particle has local map initialized -> robot placed perfectly in the center of local matrix but no data
#look at resize method in gabecode.py and run this resize method in the particle. tbh this method should just be moved to this class...


#methods:
#####particle moves with velocity model plus some error (sample velocity model)


#####particle takes a measurement with all sensors and back-calculates likelihood of position (measurement model map)
#here is where we write the sensor model -> use several spaced out ultrasound sensors, and these sensors are described as:
# the correct algorithm for range finder sensor modelling is on book pg 172 (this is too simple)
# localization using an ultrasound sensor is on pg 288 -> inverse problem (I think this is the right one)
# I think we want a combo of pg 301 algorithm plus the dummy pg 172 one for just the mapping... (this is wrong but I dont want to lose page)
#book has their robot cone opening 15 degrees
#based on pg 303 it's not unusual to have a robot with evenly spaced sensors on all sides


#####based on measurements particle updates its own local map accordingly


#ORDER MATTERS so each method should probably proc the next method
#use pg 478 as a reference for an overview of the full algorithm

#### end class particle ####





#Third, I force robot to go through certain poses and make sure that the robot local map updates properly




































    

