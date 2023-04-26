from math import hypot


EUCLIDEAN = 0


class Point():
    '''Creates a point on a coordinate plane with values x and y.'''

    heuristic_type: int = -1
    goal = None

    def __init__(self, x: float, y: float, weight: float = 0):
        '''Defines x and y variables'''
        self.x = x
        self.y = y
        self.weight = weight
        self.parent = None

    def __str__(self) -> str:
        return f'({self.x}, {self.y}), weight = {self.weight}'

    def distance(self, other) -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return round(hypot(dx, dy), 4)

    def get_path(self) -> str:
        path_str = ""
        node = self
        while node is not None:
            path_str = node.__str__()
            node = node.parent
        return path_str

    # def midpoint(self, other):
    #     midx = self.x - other.x/2
    #     midy = self.y - other.y/2
    #     return Point(midx, midy)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)

    def heuristic(self) -> float:
        if self.heuristic_type == EUCLIDEAN:
            return self.euclidean()
        else:
            raise ValueError("No heuristic match found")

    def euclidean(self):
        if self.goal is None:
            raise ValueError("Goal not set")
        return self.distance(Point.goal)

    def __lt__(self, other):
        f1 = self.weight + self.heuristic()
        f2 = other.weight + other.heuristic()
        return f1 < f2
