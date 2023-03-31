import numpy as np
from math import sin,cos,pi


def measure(map,pose):
    '''measurement system for a series of range finders, see likelihood_field_range_finder_model on pg 172'''
    #

    # 6 sensors on the robot
    k=6

    

    #initial probability is 1
    q=1

    for i in range(k):
        pass





    #sensors are at multiples of 2pi/6
    pass

'''
things to add to config.yaml:
the width of the sensor (sensorwidth) in radians
rmax
dr
dtheta
'''

#This wont work and needs testing!! I have no idea what to use for input values so tune this up
def ideal_measure(map,pose,rmax,dr,dtheta):

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

    return rout
