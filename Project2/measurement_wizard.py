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

    def __init__(self, real_pose, maze):
        """Initialize variables"""

        self.ideal=Particle()
        self.ideal.setpose(real_pose)

        self.maze=maze

    def navigate_maze(self,u):
        """When passed a trajectory u vector this method will return a set of measurements z"""

        #move particle along trajectory
        self.ideal.sample_motion_model_velocity() #this not working either

        #find ideal case for measurement

        #find the real measurements that should be returned according to PDF

        #export measurement vector
        z=0
        return z
        
    def add_error_to_measurement(map,pose):
        '''measurement system for a series of range finders, see likelihood_field_range_finder_model on pg 172'''
        # fix up the ideal measurement method and put those values in here. 
        # first line of this should call ideal_measure() and the rest just samples probability

        #this probability is described on pg 171. 

        #generate the pdf for that probability and then do a probabilistic sampling of the function to get an output sensor measurement with noise


    def ideal_measure(map,pose,rmax,dr,dtheta): #This wont work and needs testing!! input values are gonna need some help so tune this up

        #Import all thetas that have sensors
        #do the math to calculate the phi values associated with the r

        '''
        # 6 sensors on the robot
        k=6

        #initial probability is 1
        q=1

        for i in range(k):
            pass

        #sensors are at multiples of 2pi/6
        pass
        '''

        r_steps=int(rmax/dr)
        theta_steps=int(pose[2]/dtheta)

        rout=rmax

        #fix this so it starts from one side fo the theta in question and goes over the whole space
        for i in range(theta_steps):
            for j in range(r_steps):
                theta=pose[2]+i*dr
                r=j*dr

                x=pose[1]+r*cos(theta)
                y=pose[0]+r*sin(theta)

                #go through every possible r, theta position in this wedge and find the smallest possible r
                if (map[y,x]==1):
                    if (r<rout):
                        rout=r
                    break

        return rout #This needs to be assembled into a z vector!

