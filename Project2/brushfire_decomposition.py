from point import Point
from math import floor
from typing import List
from collections import deque

GOAL = 2
RESOLUTION = 0.1


class Brushfire():

    def __init__(self, boundary, vertices):

        self.boundary = boundary
        self.vertices = vertices

        self.map: List[List[int]] = self.generate_map()

    def generate_map(self) -> List[List[int]]:
        # make a 2d numpy array that's a map of the space
        # put a 1 where the object is and a zero everywhere else

        return [[0, 0, 0, 0, 0],
                [1, 1, 1, 1, 0],
                [0, 0, 0, 0, 0],
                [0, 1, 1, 1, 1],
                [0, 0, 0, 0, 0]]

    def generate_map(self):
        #make a 2d list that will fit the whole space
        numPts=len(self.boundary)

        xmax=xmin
        for i in range(numPts):






        #put a 1 where the object is and a zero everywhere else -> needs mike's "detect object" code
        return None

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

        frontier = 1

        zeroes_left = True

        while zeroes_left:

            curr_q: deque[tuple[int, int]] = deque()
            curr_q.append((y, x))
            frontier += 1

            try:
                for y_list in self.map:
                    for val in y_list:
                        if val == 0:
                            raise KeyError
                zeroes_left = False
            except KeyError:
                pass

        print(self.map)

    def get_neighbors(self):
        pass

    def run(self, start: Point):
        # sum the wavefront and brushfire
        # find the steepest decline in numbers but stay away from 1s because those are obstacles
        pass


def test():
    Brushfire(1, 2).wavefront(Point(5, 0))


if __name__ == '__main__':
    test()
