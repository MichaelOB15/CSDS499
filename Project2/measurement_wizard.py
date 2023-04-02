import numpy as np
from math import sin,cos,pi
from particle import Particle

'''
things to add to config.yaml:
the width of the sensor (sensorwidth) in radians
rmax
dr
dtheta
all of the z probability values that have to add up to 1 in the method above
'''

#should this be an object?? make it measurementWizard or something and then it can hold the ideal particle and the maze

#we should probably go through this at the end and fix up my weird naming convention etc idk what the standard is

class MeasurementWizard:
    """This object handles measurement taking as the real world should. It will never spit out coordinates, but it holds 
    a single particle that knows its position in the map, and the motion of this separate particle is considered to be
    the "real" position of the robot. """

    def __init__(self, maze, real_pose):
        """Initialize variables"""

        self.ideal=Particle()
        self.ideal.setpose(real_pose)

        self.maze=maze

    def navigate_maze(self,u): #work on this last
        """When passed a trajectory u vector this method will return a set of measurements z"""

        #move particle along trajectory
        self.ideal.sample_motion_model_velocity(u)

        #find ideal case for measurement

        #find the real measurements that should be returned according to PDF

        #export measurement vector
        z=0
        return z
        
    def add_error_to_measurement(map,pose): #fix ideal case first
        '''measurement system for a series of range finders, see likelihood_field_range_finder_model on pg 172'''
        # fix up the ideal measurement method and put those values in here. 
        # first line of this should call ideal_measure() and the rest just samples probability

        #this probability is described on pg 171. 

        #generate the pdf for that probability and then do a probabilistic sampling of the function to get an output sensor measurement with noise


    def ideal_measure(self,rmax,dr,dtheta): #This wont work and needs testing!! input values are gonna need some help so tune this up

        #Import all thetas that have sensors
        #do the math to calculate the phi values associated with the r
        map=self.maze
        pose=self.ideal.getpose()

        # 6 sensors on the robot
        num_sensors=6

        # each sensor is pointing in equidistant directions from 0 to 2pi
        theta=np.zeros(num_sensors)
        for i in range(num_sensors):
            theta[i]=i*2*pi/num_sensors
        
        r_steps=int(rmax/dr)
        theta_steps=int(pose[2]/dtheta) #beam width code implemented here??

        rout=np.zeros(num_sensors)+rmax

        for s in range(num_sensors):

            #fix this so it starts from one side for the theta in question and goes over the whole space
            for i in range(theta_steps):
                for j in range(r_steps):
                    theta=theta[s]+pose[2]+i*dtheta
                    r=j*dr

                    #this doesnt work yet :( I need to put a beam width on my sensor and it goes in this block here

                    x=pose[1]+r*cos(theta[s])
                    y=pose[0]+r*sin(theta[s])

                    #go through every possible r, theta position in this wedge and find the smallest possible r
                    if (map[y,x]==1):
                        if (r<rout[s]):
                            rout[s]=r
                        break

        return np.concatenate([rout,theta],axis=0) #z vector

