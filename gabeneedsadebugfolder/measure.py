import numpy as np


def measure(map,pose):
    '''measurement system for a series of range finders, see likelihood_field_range_finder_model on pg 172'''

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
dr
dtheta
'''

'''
def ideal_measure(map,pose):
    for i in range(pose[2])
    pass
'''