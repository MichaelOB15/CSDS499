from math import hypot


class Point():
    '''Creates a point on a coordinate plane with values x and y.'''

    def __init__(self, x: float, y: float):
        '''Defines x and y variables'''
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f'({self.x}, {self.y})'

    def distance(self, other) -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return hypot(dx, dy)

    def midpoint(self, other):
        midx = self.x - other.x/2
        midy = self.y - other.y/2
        return Point(midx, midy)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)
