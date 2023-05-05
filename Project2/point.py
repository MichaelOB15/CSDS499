from math import hypot
from typing import Optional


EUCLIDEAN = 0


class Point():
    '''Creates a point on a coordinate plane with values x and y.'''

    heuristic_type: int
    goal: 'Point'

    def __init__(self, x: float, y: float, weight: float = 0):
        '''Defines x and y variables'''
        self.x = x
        self.y = y
        self.weight = weight
        self.parent: Optional['Point'] = None

    def __str__(self) -> str:
        return f'({self.x}, {self.y}), weight = {self.weight}'

    def distance(self, other: 'Point') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return round(hypot(dx, dy), 4)

    def get_path(self) -> str:
        path_str = self.__str__()
        self = self.parent
        while self is not None:
            path_str = f'{path_str}, {self}'
            self = self.parent
        return path_str

    def heuristic(self) -> float:
        if self.heuristic_type == EUCLIDEAN:
            return self.euclidean()
        else:
            raise ValueError("No heuristic match found")

    def euclidean(self):
        if self.goal is None:
            raise ValueError("Goal not set")
        return self.distance(Point.goal)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other: 'Point'):
        return (self.x, self.y) == (other.x, other.y)

    def __lt__(self, other: 'Point'):
        f1 = self.weight + self.heuristic()
        f2 = other.weight + other.heuristic()
        return f1 < f2
