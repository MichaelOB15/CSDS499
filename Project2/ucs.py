from collections import deque
from typing import List, Tuple, Any, Union, Dict
from math import floor
from visualize import Visualization
from robot_motion import Robot_motion
import numpy as np
from math import sin, cos, pi, sqrt
from maze import Maze


# TODO from params?
UNEXPLORED = 0.5
STEP = 1
WALL = 1
OPEN = 0


class Node:
    def __init__(self, parent, pos, cost):
        self.parent: Union[Node, None] = parent
        self.pos: Tuple[float, float, float] = pos
        self.cost: int = cost

    def get_children(self, m, explored: Dict) -> List[Any]:
        children: List[Node] = []
        y = self.pos[0]
        x = self.pos[1]

        if y - 1 >= 0:
            if m[y - 1][x] != WALL:
                children.append(Node(self, (y - 1, x, pi/2), self.cost + 1))

        if y + 1 < len(m):
            if m[y + 1][x] != WALL:
                children.append(Node(self, (y + 1, x, -pi/2), self.cost + 1))

        if x - 1 >= 0:
            if m[y][x - 1] != WALL:
                children.append(Node(self, (y, x - 1, -pi), self.cost + 1))

        if x + 1 < len(m[x]):
            if m[y][x + 1] != WALL:
                children.append(Node(self, (y, x + 1, 0), self.cost + 1))

        remove_explored: List[Node] = []
        for child in children:
            if explored.get(child.pos) is None:
                remove_explored.append(child)

        return remove_explored

    def get_motion(self, delta_t) -> List[int]:
        p1 = self.pos
        p2 = self.parent.pos

        x_hat = p2[1] - p1[1]
        y_hat = p2[0] - p1[0]
        theta_hat = p2[2] - p1[2]

        w = theta_hat/delta_t

        if w == 0:
            v = sqrt(x_hat**2 + y_hat**2)
        else:
            # bottom_v1 = -sin(p1[1])+sin(p1[1]+w*delta_t)
            # bottom_v2 = cos(p1[1])-cos(p1[1]+w*delta_t)
            # v1 = x_hat*w/(bottom_v1)
            # v2 = y_hat*w/(bottom_v2)
            # # print("Turning :(")

            # if x_hat == 0:
            #     v = v1
            # elif y_hat == 0:
            #     v = v2

            v = sqrt(x_hat**2 + y_hat**2)
            return [[0, w], [v, 0]]

        return [v, w]

    def __str__(self) -> str:
        return f'y = {self.pos[0]}, x = {self.pos[1]}, theta = {self.pos[2]},  cost = {self.cost}'


# Returns index of nearest UNEXPLORED
def nearest_unexplored(m: List[List[int]], pos: List[int]) -> Node:
    robot_x_in_m = floor(pos[0])
    robot_y_in_m = floor(pos[1])
    robot_theta = pos[2]
    pos = (robot_y_in_m, robot_x_in_m, robot_theta)

    explored: Dict[Tuple[int, int, int], Node] = {}

    queue: deque[Node] = deque()
    queue.append(Node(None, pos, 0))
    explored.update({pos: Node(None, pos, 0)})
    while len(queue) >= 1:

        node = queue.popleft()
        if m[node.pos[0]][node.pos[1]] == UNEXPLORED:
            return node
        else:
            children: List[Node] = node.get_children(m, explored)
            for child in children:
                queue.append(child)
                explored.update({child.pos: child})

    # TODO add check -1 remains
    raise ValueError("No unexplored space found")


def generate_discrete_path(node: Node, delta_t: int) -> List[List[int]]:
    motions: List[List[int]] = []
    while node.parent is not None:
        motion = node.get_motion(delta_t)
        if type(motion[0]) != float:
            motions.insert(0, motion[1])
            motions.insert(0, motion[0])
        else:
            motions.insert(0, motion)
        node = node.parent

    return motions


# m = [[OPEN, OPEN, OPEN, OPEN, OPEN], [OPEN, WALL, WALL, WALL, OPEN],  [OPEN, WALL, OPEN, OPEN, OPEN], [OPEN, WALL, WALL, WALL, WALL], [OPEN, OPEN, OPEN, OPEN, UNEXPLORED]]

# m = np.array([[WALL, WALL, WALL, WALL], [WALL, WALL, WALL, WALL], [UNEXPLORED, OPEN, OPEN, OPEN], [WALL, WALL, WALL, WALL]])
# start_pos = np.array([3, 2, 0])

nx, ny = 3, 3
# Maze entry position
ix, iy = 0, 0

# allows passageways to have widths
scaling = 4

maze = Maze(nx, ny, scaling, ix, iy)
maze.make_maze()


# m is 2D matrix
m = maze.out()
for i in range(len(m)):
    for j in range(scaling):
        m[i][j] = UNEXPLORED
print(m)

start_pos = [floor(len(m[0])/2) - 0, floor(len(m)/2) - 0, 0]

viz = Visualization(m, start_pos)
# a = np.zeros([1, 6])
a = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

node = nearest_unexplored(m, start_pos)

delta_t = 1

discrete_path = generate_discrete_path(node, delta_t)
print(discrete_path)
x_t_next = start_pos

counter = 0

for command in discrete_path:
    # TODO walls lmao
    x_t_next = Robot_motion(command, x_t_next, a, delta_t, m, 0.01).actual_motion_model_velocity()
    print(x_t_next)
    # if counter % 5 == 0:
    viz.update(m, x_t_next, counter % 5 == 0)
    counter = counter + 1

viz.update(m, x_t_next)
