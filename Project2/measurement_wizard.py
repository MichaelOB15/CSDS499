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

    def getpose(self):
        return self.ideal.getpose()

    def navigate_maze(self,u,a,stepsize): #work on this last and get these inputs in the config
        """When passed a trajectory u vector this method will return a set of measurements z"""

        #move particle along trajectory
        self.ideal.sample_motion_model_velocity(u,a,stepsize) #get a and stepsize out of the method call and into config :(

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


    def ideal_measure(self,rmax,dr,dtheta):
        
        map=self.maze
        pose=self.ideal.getpose()

        # 6 sensors on the robot, put this in config
        num_sensors=6

        # each sensor is pointing in equidistant directions from 0 to 2pi
        theta=np.zeros(num_sensors)
        for i in range(num_sensors):
            theta[i]=i*2*pi/num_sensors
        
        beam_width = 15 #15 degree beam width, this should probably go in the config file
        spread = beam_width*pi/180

        r_steps=int(rmax/dr)
        theta_steps=int(spread/dtheta)

        rout=np.zeros(num_sensors)+rmax

        for s in range(num_sensors):

            for i in range(theta_steps):
                for j in range(r_steps):

                    temp_angle=pose[2]+theta[s]-spread/2+i*dtheta
                    temp_r=j*dr

                    x=int(pose[1]+temp_r*cos(temp_angle))
                    y=int(pose[0]+temp_r*sin(temp_angle))

                    #go through every possible r, theta position in this wedge and find the smallest possible r
                    if (map[y,x]==1):
                        if (temp_r<rout[s]):
                            rout[s]=temp_r
                        break

                    #Im using this for testing-- delete later!! this will seriously mess things up but it makes the map boxes grey
                    map[y,x]=-1

        return np.concatenate([rout,theta+pose[2]],axis=0) #z vector

