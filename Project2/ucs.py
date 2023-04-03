from collections import deque
from typing import List, Tuple, Any, Union, Dict
from math import floor
from visualize import Visualization
from robot_motion import Robot_motion, in_wall
from math import pi, sqrt
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
            if not in_wall(m, (x, y - 1)):
                children.append(Node(self, (y - 1, x, pi/2), self.cost + 1))

        if y + 1 < len(m):
            if not in_wall(m, (x, y + 1)):
                children.append(Node(self, (y + 1, x, -pi/2), self.cost + 1))

        if x - 1 >= 0:
            if not in_wall(m, (x - 1, y)):
                children.append(Node(self, (y, x - 1, -pi), self.cost + 1))

        if x + 1 < len(m[x]):
            if not in_wall(m, (x + 1, y)):
                children.append(Node(self, (y, x + 1, 0), self.cost + 1))

        remove_explored: List[Node] = []
        for child in children:
            if explored.get(child.pos) is None:
                remove_explored.append(child)

        return remove_explored

    def get_motion(self, next_node: Any, delta_t) -> List[float]:
        p2 = self.pos
        p1 = next_node.pos

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
        return f'x = {self.pos[1]}, y = {self.pos[0]}, theta = {self.pos[2]},  cost = {self.cost}'


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
    while queue:

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


# def generate_discrete_path(node: Node, delta_t: int) -> List[List[int]]:
#     motions: List[List[int]] = []
#     while node.parent is not None:
#         motion = node.get_motion(delta_t)
#         if type(motion[0]) != float:
#             motions.insert(0, motion[1])
#             motions.insert(0, motion[0])
#         else:
#             motions.insert(0, motion)
#         node = node.parent

#     return motions


# m = [[OPEN, OPEN, OPEN, OPEN, OPEN], [OPEN, WALL, WALL, WALL, OPEN],  [OPEN, WALL, OPEN, OPEN, OPEN], [OPEN, WALL, WALL, WALL, WALL], [OPEN, OPEN, OPEN, OPEN, UNEXPLORED]]

# m = np.array([[WALL, WALL, WALL, WALL], [WALL, WALL, WALL, WALL], [UNEXPLORED, OPEN, OPEN, OPEN], [WALL, WALL, WALL, WALL]])
# start_pos = np.array([3, 2, 0])

nx, ny = 10, 10
# Maze entry position
ix, iy = 0, 0

# allows passageways to have widths
scaling = 8

maze = Maze(nx, ny, scaling, ix, iy)
maze.make_maze()


# m is 2D matrix
m = maze.out()
for i in range(scaling, 2*scaling):
    for j in range(scaling):
        m[i][j] = UNEXPLORED
# print(m)

start_pos = [floor(len(m[0])/2) + scaling, floor(len(m)/2) + scaling, 0]

viz = Visualization(m, start_pos)

# a = np.zeros([1, 6])
# a = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
a = [0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001]

node = nearest_unexplored(m, start_pos)

node_list: List[Node] = []

tmp = node

# Flips path
while tmp is not None:
    node_list.insert(0, tmp)
    tmp = tmp.parent

for node in node_list:
    print(node)

delta_t = 1
x_t_next = start_pos

# while UNEXPLORED in m:
for i in range(len(node_list) - 1):
    motions = node_list[i].get_motion(node_list[i + 1], delta_t)

    if type(motions[0]) == float:

        x_t_next = Robot_motion(motions, x_t_next, a, delta_t, m, 0.01).actual_motion_model_velocity()
        # print(x_t_next)
        # if counter % 5 == 0:
        viz.update(m, x_t_next, i % 40 == 0)
    else:
        # print(motions)
        x_t_next = Robot_motion(motions[0], x_t_next, a, delta_t, m, 0.01).actual_motion_model_velocity()
        x_t_next = Robot_motion(motions[1], x_t_next, a, delta_t, m, 0.01).actual_motion_model_velocity()

viz.update(m, x_t_next)
viz.pause()
