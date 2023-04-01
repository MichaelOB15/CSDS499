import numpy as np
from math import sin,cos,pi

'''
things to add to config.yaml:
the width of the sensor (sensorwidth) in radians
rmax
dr
dtheta
all of the z probability values that have to add up to 1 in the method above
'''

def measure(map,pose):
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

