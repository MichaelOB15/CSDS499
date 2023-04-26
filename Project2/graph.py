from typing import Dict, List, Union
from point import Point, EUCLIDEAN
import matplotlib.pyplot as plt
from queue import PriorityQueue


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
        self.g[p1].append(Point(p2.x, p2.y, p1.distance(p2)))
        self.g[p2].append(Point(p1.x, p1.y, p2.distance(p1)))

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

    def a_star(self, start: Point, goal: Point, heuristic_type: int) -> Point:

        Point.heuristic_type = heuristic_type
        Point.goal = goal

        explore: Dict[Point, bool] = {}
        q = PriorityQueue()
        q.put(start)
        explore[start] = True

        while not q.empty():
            front = q.get()
            if front == goal:
                return goal
            else:
                children = self.get_neighbors(front)
                for child in children:
                    if not explore.get(child, False):
                        child.parent = front
                        q.put(child)
                        explore[child] = True

        raise KeyError(f"No goal point = {goal} found")


g = Graph()
g.add_node(0, 0)
g.add_node(Point(1, 0))
g.add_node(1, 1)
g.add_node(0, 1)


g.add_vertex(Point(1, 0), Point(0, 1))
g.add_vertices(Point(1, 1), [Point(0, 0), Point(0, 1), Point(1, 0)])

print(g)

path_node = g.a_star(Point(1, 0), Point(0, 1), EUCLIDEAN)
g.graph_vis()

print(path_node.get_path())
