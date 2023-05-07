from point import Point
from math import floor
from typing import List, Tuple
from collections import deque

GOAL = 2
RESOLUTION = 0.1


class Brushfire():

    def __init__(self, boundary, vertices):

        self.boundary = boundary
        self.vertices = vertices

        self.map: List[List[int]] = self.generate_map()

    def generate_map(self):
        return
        #make a 2d list that will fit the whole space
        numPts=len(self.boundary)

        xmax=ymax=-999999999999 #determine the size of the space
        xmin=ymin=999999999999
        for i in range(numPts):
            if (self.boundary[i][0]>xmax):
                xmax=self.boundary[i][0]
            if (self.boundary[i][1]>ymax):
                ymax=self.boundary[i][1]
            if (self.boundary[i][0]<xmin):
                xmin=self.boundary[i][0]
            if (self.boundary[i][1]<ymin):
                ymin=self.boundary[i][1]

        xdim=int(xmax-xmin)/RESOLUTION
        ydim=int(ymax-ymin)/RESOLUTION

        map=[]
        for x in range(xdim):
            map.append([])
            for y in range(ydim):
                map[x][y]=self.isObstacle()#put a 1 where the object is and a zero everywhere else -> needs mike's "detect object" code

        return map

    def isObstacle(self):
        return None # put a 1 where the object is and a zero everywhere else -> needs mike's "detect object" code

    def brushfireAlg(self):
        # expand the map from generate_map so each pixel holds the distance to the nearest object
        map=self.map()

        # copy the map
        copy=[]
        for x in range(len(map[0])):
            #adds a blank list
            copy.append([])

            #populates the new list
            for y in range(len(map[1])):
                copy[x].append(map[x][y])

        map=copy

        #number of times this process runs-- worst case runtime is the length of the map
        iter=len(map)
        if (len(map[0]>iter)):
            iter=len(map[0])

        while (iter>0):

            # visit every pixel in the map
            for x in range(len(map)):
                for y in range(len(map[0])):

                    # if the value is zero it hasn't been touched
                    if(not map[x][y]==0):

                        # update all eight adjacent squares
                        for a in range(3):
                            for b in range(3):
                                xind=x-1+a
                                yind=y-1+b

                                # can't go out of bounds
                                if (xind>len(map)):
                                    continue
                                if (yind>len(map[0])):
                                    continue

                                #can't let [x,y] query
                                if (xind==x and yind==y):
                                    continue

                                #target is already filled with data
                                if (not map[xind][yind]==0):
                                    continue

                                #update the target
                                map[xind][yind]=map[x][y]+1
            iter=iter-1
        return map

    def wavefront(self, goal: Point):
        x = floor(goal.x) - 1
        y = len(self.map) - floor(goal.y) - 1

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

        print(self.map)

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
    bf = Brushfire(1, 2)
    bf.map = [[0, 0, 0, 0, 0],
              [1, 1, 1, 1, 0],
              [0, 0, 0, 0, 0],
              [0, 1, 1, 1, 1],
              [0, 0, 0, 0, 0]]
    bf.wavefront(Point(5, 0))


if __name__ == '__main__':
    test()
