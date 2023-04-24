from typing import Dict, List, Union
from point import Point
import matplotlib.pyplot as plt


class Graph():

    def __init__(self):
        self.g: Dict[Point, List[Point]] = {}

    def add_node(self, point_or_x: Union[Point, float], y: float = -1):
        if isinstance(point_or_x, Point):
            self.g[point_or_x] = []
        else:
            self.g[Point(point_or_x, y)] = []

    def add_vertex(self, p1: Point, p2: Point):
        self.g[p1].append(p2)
        self.g[p2].append(p1)

    def add_vertexes(self, parent: Point, children: List[Point]):
        for p in children:
            self.add_vertex(parent, p)

    def __str__(self) -> str:
        return self.g.__str__()

    def graph_vis(self):

        points = self.g.keys()
        x: List[float] = []
        y: List[float] = []
        for p in points:
            x.append(p.x)
            y.append(p.y)

        plt.scatter(x, y)
        plt.show()

g = Graph()
g.add_node(0, 0)
g.add_node(1, 0)
g.add_node(1, 1)
g.add_node(0, 1)

g.graph_vis()
