import numpy as np
from math import sqrt, atan2, sin, cos


def fillBoundary(xArray, yArray):
    '''add points in a line between consecutive "too distant" boundary vertices on the obstacle polygon'''

    resolution = 0.1

    length = np.length(xArray)  # assume the arrays are the same length, this method has no error handling

    xUpdate = np.array()  # blank arrays that will fill with data
    yUpdate = np.array()

    for i in range(length-1):
        x1 = xArray(i)
        x2 = xArray(i+1)
        y1 = yArray(i)
        y2 = yArray(i+1)

        dist = sqrt((x2-x1)^2 + (y2-y1)^2)  # euclidean distance between vertices
        angle = atan2((y2-y1), (x2-x1))  # angle from point to next point

        numAdditions = int(dist/resolution)+1  # run loop for zero case to add original vertex + as many more times as necessary

        for j in range(numAdditions):
            xUpdate.append(x1+j*cos(angle)*resolution)
            yUpdate.append(y1+j*sin(angle)*resolution)

    return (xUpdate, yUpdate)  # return as a tuple


class Obstacle():
    '''An obstacle out of a set of points x and y.'''

    def __init__(self, xArray, yArray):
        '''Defines x and y variables'''

        # Two parallel arrays that are the set of points x,y that make up the boundary.
        tup = fillBoundary(xArray, yArray)
        self.xArray = tup[0]
        self.yArray = tup[1]
        self.pts = np.length(xArray)  # max index of the x and y parallel arrays

    def pointInPolygon(self, point):
        '''implementation of ray casting algorithm. Returns boolean.'''

        # code needs to "latch" so successive boundary hits don't count as a bunch of boundary walls
        latch = False

        return False
