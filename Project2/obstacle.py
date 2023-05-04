import numpy as np
from math import sqrt,atan2,sin,cos,pi

def fillBoundary(vertices):
    '''add points in a line between consecutive "too distant" boundary vertices on the obstacle polygon'''

    resolution=0.1

    l=np.length(vertices) #assume the arrays are the same length, this method has no error handling

    xUpdate=np.array() #blank arrays that will fill with data
    yUpdate=np.array()

    for i in range(l-1):
        
        x1=vertices[i][0]
        x2=vertices[i+1][0]
        y1=vertices[i][1]
        y2=vertices[i+1][1]

        dist=sqrt((x2-x1)^2+(y2-y1)^2) #euclidean distance between vertices
        angle=atan2((y2-y1),(x2-x1)) #angle from point to next point

        numAdditions=int(dist/resolution)+1 #run loop for zero case to add original vertex + as many more times as necessary

        for j in range(numAdditions):
            xUpdate.append(x1+j*cos(angle)*resolution)
            yUpdate.append(y1+j*sin(angle)*resolution)

    return (xUpdate,yUpdate) #return as a tuple

class Obstacle():
    '''An obstacle out of a set of points x and y.'''

    # Original data passed to the Obstacle class
    vertices=None

    # Two parallel arrays that are the set of points x,y that make up the boundary. 
    xArray=None
    yArray=None

    # Absolute boundary of the obstacle
    xmax=None
    xmin=None
    ymax=None
    ymin=None

    def __init__(self, vertices):
        '''Defines x and y variables'''

        tup=fillBoundary(vertices) #adds in extra points

        xmax=float('-inf')
        xmin=float('inf')
        ymax=float('-inf')
        ymin=float('inf')

        l=np.length(self.vertices)

        #finds the bounds of the obstacle
        for i in range(l): 
            if self.vertices[l][0]<xmin:
                xmin=self.vertices[l][0]
            if self.vertices[l][0]>xmax:
                xmin=self.vertices[l][0]
            if self.vertices[l][1]<ymin:
                ymin=self.vertices[l][1]
            if self.vertices[l][1]>ymax:
                ymax=self.vertices[l][1]

        self.vertices=vertices
        self.xArray = tup[0]
        self.yArray = tup[1]

    def pointInPolygon(self,point):
        '''implementation of ray casting algorithm. Returns boolean.'''

        #code needs to "latch" so successive boundary hits don't count as a bunch of boundary walls
        latch=False

        resolution=0.1

        x1=point[0]
        y1=point[1]
        x2=self.xArray[0]
        y2=self.yArray[0]

        angle=atan2((y2-y1),(x2-x1))

        #determine which dimension hits the boundary first
        ##This is not optimal but still should do the trick
        if(angle>(3*pi/4) | angle<(-3*pi/4)):
            numIterations=int((x1-self.xmin)/resolution)
        elif(angle>pi/4):
            numIterations=int((self.ymax-y1)/resolution)
        elif(angle>(-pi/4)):
            numIterations=int((self.xmax-x1)/resolution)
        else:
            numIterations=int((y1-self.ymin)/resolution)

        for i in range(numIterations):
            
            xUpdate.append(x1+j*cos(angle)*resolution)
            yUpdate.append(y1+j*sin(angle)*resolution)


        return False
    


    





