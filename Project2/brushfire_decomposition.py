from point import Point
from math import floor
from typing import List, Tuple
from collections import deque
import matplotlib.pyplot as plt
import numpy as np

GOAL = 2
RESOLUTION = 0.1


class Brushfire():
    boundary=None
    obstacles=None
    start=None
    end=None

    # represent coordinates/map offset and will need to be calculated back into any output
    xmax=float('-inf')
    xmin=float('inf')
    ymax=float('-inf')
    ymin=float('inf')

    rays=None
    critical_points=None

    def __init__(self, boundary, obstacles, start, end):

        self.boundary = boundary
        self.obstacles = obstacles
        self.start = start
        self.end = end

        #set up critical points and rays
        critical_points = []
        rays = []

        last_point = self.boundary[-1]
        for point in self.boundary:
            critical_points.append(point)

            rays.append([last_point, point])
            last_point = point

        for obstacle in self.obstacles:

            last_point = obstacle[-1]
            for point in obstacle:
                critical_points.append(point)

                rays.append([last_point, point])
                last_point = point

        self.critical_points=critical_points
        self.rays=rays

        self.map: List[List[int]] = self.generate_map()  # updates max/min as well
                

    def generate_map(self):
        #make a 2d list that will fit the whole space

        for point in self.boundary:
            if (point[0]>self.xmax):
                self.xmax=point[0]
            if (point[1]>self.ymax):
                self.ymax=point[1]
            if (point[0]<self.xmin):
                self.xmin=point[0]
            if (point[1]<self.ymin):
                self.ymin=point[1]

        xdim=int((self.xmax-self.xmin)/RESOLUTION)
        ydim=int((self.ymax-self.ymin)/RESOLUTION)

        map=[]
        for x in range(xdim):
            map.append([])
            for y in range(ydim):
                #put a 1 where the object is and a zero everywhere else -> needs mike's "detect object" code
                map[x].append(self.isObstacle([x*RESOLUTION+self.xmin,y*RESOLUTION+self.ymin]))

        return map


    def isObstacle(self,point):
        '''tell whether a point is in an obstacle'''
        rays=self.rays

        try:
            num_intersections = 0

            for ray in rays:
                if ray[0][0] < ray[1][0]:
                    xmin = ray[0][0]
                    xmax = ray[1][0]
                else:
                    xmin = ray[1][0]
                    xmax = ray[0][0]

                if ray[0][0] < ray[1][0]:
                    first = ray[0]
                    second = ray[1]
                else:
                    first = ray[1]
                    second = ray[0]

                m = (second[1]-first[1]) / (second[0]-first[0])

                b = first[1] - (m * first[0]) 

                new_y = (m * point[0]) + b

                point_on_ray = [point[0], new_y]

                # in the boundary and above
                if point[0] < xmax and point[0] > xmin and point_on_ray[1] > point[1]:
                    num_intersections += 1

            # if odd num of interactions
            if num_intersections % 2:
                return 0
            else:
                return 1
        except:
            return 1
        

    def brushfireAlg(self):
        '''
        map=self.map

        # copy the map
        copy=np.zeros((len(self.map),len(self.map[0])))
        for x in range(len(self.map[0])):
            for y in range(len(self.map[1])):
                #goes from List to numpy array
                copy[x,y]=self.map[x][y]

        #number of times this process runs-- worst case runtime is the length of the map
        iter=len(self.map)
        if (len(self.map[0])>iter):
            iter=len(self.map[0])

        map=copy

        while (iter>0):
            print(iter)

            # visit every pixel in the map
            for x in range(len(self.map)):
                for y in range(len(self.map[0])):

                    # if the value is zero it hasn't been touched
                    if(not map[x,y]==0):

                        # update all eight adjacent squares
                        for a in range(3):
                            for b in range(3):
                                xind=x-1+a
                                yind=y-1+b

                                # can't go out of bounds
                                if (xind>=len(self.map)):
                                    continue
                                if (yind>=len(self.map[0])):
                                    continue

                                #can't let [x,y] query
                                if (xind==x and yind==y):
                                    copy[x,y]=map[x,y]
                                    continue

                                #target is already filled with data
                                if (not map[xind,yind]==0):
                                    #target data is from the map
                                    if (not map[xind,yind]==0):
                                        copy[xind,yind]=map[xind,yind]
                                        continue
                                    #target holds a higher number
                                    if (copy[xind,yind]>map[x,y]+1):
                                        copy[xind,yind]=map[x,y]+1
                                    continue

                                #update the target
                                copy[xind,yind]=map[x,y]+1
                                

            map=copy
            copy=np.zeros((len(self.map),len(self.map[0])))
            #np.savetxt("foo.csv", map, delimiter=",")
            iter=iter-1
        return map
        '''
        oldmap=np.zeros((len(self.map),len(self.map[0])))
        newmap=np.zeros((len(self.map),len(self.map[0])))
        nodes=[]
        for x in range(len(self.map)):
                for y in range(len(self.map[0])):
                    nodes.append([x,y])
                    oldmap[x,y]=self.map[x][y]

        while(len(nodes)>0):
            print(len(nodes))
            iter=len(nodes)
            for i in reversed(range(iter)):
                x1=nodes[i][0]
                y1=nodes[i][1]

                if(not oldmap[x1,y1]==0):
                    nodes.remove([x1,y1])
                    
                    #populate new map with surrounding nodes
                    for a in range(3):
                        for b in range(3):
                            if(oldmap[a,b]==0):
                                if(newmap[a,b]<oldmap[x1,y1]+1):
                                    newmap[a,b]=oldmap[x1,y1]+1
                            
            
            oldmap=newmap.copy()
            



    def wavefront(self):
        goal = Point(self.end[0], self.end[1])
        x = (floor(goal.x/RESOLUTION) - 1)
        y = (len(self.map) + floor(goal.y/RESOLUTION) - 1)
        self.map[y][x] = GOAL

        frontier = 3

        curr_q: deque[tuple[int, int]] = deque()
        next_q: deque[tuple[int, int]] = deque()
        curr_q.append((y, x))

        self.explored = {(y, x): True}

        while len(curr_q) != 0:
            while len(curr_q) != 0:
                neighbors = self.get_and_update_neighbors(frontier, curr_q.popleft())

                for neighbor in neighbors:
                    self.explored[neighbor] = True
                    next_q.append(neighbor)

            curr_q = next_q.copy()
            next_q = deque()

            frontier += 1

    def get_and_update_neighbors(self, frontier: int, point: Tuple[int, int]) -> List[Tuple[int, int]]:
        neighbors = []
        (y, x) = point

        if (y - 1) >= 0 and self.map[y - 1][x] == 0:
            if not self.explored.get((y - 1, x), False):
                self.map[y - 1][x] = frontier
                neighbors.append((y - 1, x))
        if (y + 1) < len(self.map) and self.map[y + 1][x] == 0:
            if not self.explored.get((y + 1, x), False):
                self.map[y + 1][x] = frontier
                neighbors.append((y + 1, x))

        if (x - 1) >= 0 and self.map[y][x - 1] == 0:
            if not self.explored.get((y, x - 1), False):
                self.map[y][x - 1] = frontier
                neighbors.append((y, x - 1))
        if (x + 1) < len(self.map[0]) and self.map[y][x + 1] == 0:
            if not self.explored.get((y, x + 1), False):
                self.map[y][x + 1] = frontier
                neighbors.append((y, x + 1))

        return neighbors

    def run(self, start: Point):



        # sum the wavefront and brushfire
        # find the steepest decline in numbers but stay away from 1s because those are obstacles
        pass

def test():
    pass
'''
    bf = Brushfire(1, 2)
    bf.wavefront(Point(5, 0))
'''

if __name__ == '__main__':
    test()
