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

    def get_nodes(self):
        return self.g.keys()

    def get_neighbors(self, point_or_x: Union[Point, float], y: float = -1) -> List[Point]:
        if isinstance(point_or_x, Point):
            return self.g[point_or_x]
        else:
            return self.g[Point(point_or_x, y)]

    def add_vertex(self, p1: Point, p2: Point):
        self.g[p1].append(p2)
        self.g[p2].append(p1)

    def add_vertices(self, parent: Point, children: List[Point]):
        for p in children:
            self.add_vertex(parent, p)

    def __str__(self) -> str:
        string: str = ""
        for n in self.get_nodes():
            string = string + f"{n} : ["
            for neigbors in self.get_neighbors(n):
                string = string + f'{neigbors}, '
            string = string + "]\n"
        return string

    def graph_vis(self):

        x: List[float] = []
        y: List[float] = []
        for p in self.get_nodes():

            for neighbors in self.get_neighbors(p):
                plt.plot([p.x, neighbors.x], [p.y, neighbors.y])

            x.append(p.x)
            y.append(p.y)

        plt.scatter(x, y)
        plt.show()


g = Graph()
g.add_node(0, 0)
g.add_node(Point(1, 0))
g.add_node(1, 1)
g.add_node(0, 1)


g.add_vertex(Point(1, 0), Point(0, 1))
g.add_vertices(Point(1, 1), [Point(0, 0), Point(0, 1), Point(1, 0)])

g.graph_vis()
