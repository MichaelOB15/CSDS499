from collections import deque
from typing import List, Any, Union, Dict
from math import pi, sqrt, floor, ceil


# TODO from params?
UNEXPLORED = 0.5
STEP = 1
WALL = 1
OPEN = 0


class Node:
    def __init__(self, parent, pos, cost, r, square_size):
        self.parent: Union[Node, None] = parent
        self.pos: List[float] = pos
        self.cost: int = cost
        self.r = r
        self.square_size = square_size
        self.magicval = 16

    def in_neigbhorhood(self, m) -> bool:

        # return True
        y = self.pos[0]
        x = self.pos[1]

        magicrad = self.magicval

        for j in range(y -magicrad, y + magicrad ):
            for i in range( x - magicrad, x + magicrad):
                if m[j][i] > 0.5:
                    return False
 
        return True

    def get_children(self, m, explored: Dict) -> List[Any]:
        children: List[Node] = []
        y = self.pos[0]
        x = self.pos[1]

        # val = m[y][x]

        if self.in_neigbhorhood(m):
            if y - 1 >= 0:
                children.append(Node(self, (y - 1, x, pi/2), self.cost + 1, self.r, self.square_size))

            if y + 1 < len(m):
                # if self.in_neigbhorhood(y + 1, x):
                children.append(Node(self, (y + 1, x, -pi/2), self.cost + 1, self.r, self.square_size))

            if x - 1 >= 0:
                # if not self.in_wall(m, (x - 1, y)):
                # if self.in_neigbhorhood(y, x - 1):
                children.append(Node(self, (y, x - 1, -pi), self.cost + 1, self.r, self.square_size))

            if x + 1 < len(m[0]):
                # if not self.in_wall(m, (x + 1, y)):
                # if self.in_neigbhorhood(y, x + 1):
                children.append(Node(self, (y, x + 1, 0), self.cost + 1, self.r, self.square_size))

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

        w = theta_hat/delta_t % 2 * pi

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

    # https://math.stackexchange.com/questions/2984061/cover-a-circle-with-squares
    def in_wall(self, m, pos) -> bool:
        x_in_m = floor(pos[0])
        y_in_m = floor(pos[1])

        lambdaval = self.r/self.square_size

        lambdaval = 18

        assert lambdaval >= 1

        for x in range(floor(lambdaval)):
            y = ceil(sqrt(lambdaval**2 - x**2))
            if m[y_in_m + y][x_in_m + x] == WALL:
                return True

        for x in range(floor(lambdaval)):
            y = ceil(sqrt(lambdaval**2 - x**2))
            if m[y_in_m + y][x_in_m - x] == WALL:
                return True

        for x in range(floor(lambdaval)):
            y = ceil(sqrt(lambdaval**2 - x**2))
            if m[y_in_m - y][x_in_m - x] == WALL:
                return True

        for x in range(floor(lambdaval)):
            y = ceil(sqrt(lambdaval**2 - x**2))
            if m[y_in_m - y][x_in_m + x] == WALL:
                return True

        return False


class UCS:

    def __init__(self, r, square_size):
        self.r = r
        self.square_size = square_size

    def nearest_list(self, m: List[List[int]], pos: List[int]) -> List[Node]:
        tmp: Node = self._nearest_unexplored(m, pos)

        node_list: List[Node] = []
        # Flips path
        while tmp is not None:
            node_list.insert(0, tmp)
            tmp = tmp.parent
        return node_list

    # Returns index of nearest UNEXPLORED
    def _nearest_unexplored(self, m: List[List[int]], pos: List[int]) -> Node:
        robot_y_in_m = floor(pos[0])
        robot_x_in_m = floor(pos[1])
        robot_theta = pos[2]
        pos = (robot_y_in_m, robot_x_in_m, robot_theta)

        explored: Dict[List[float], Node] = {}

        queue: deque[Node] = deque()

        n = Node(None, pos, 0, self.r, self.square_size)

        queue.append(n)
        explored.update({pos: n})
        while queue:

            node = queue.popleft()
            # print(len(m))
            # print(len(m[0]))
            # print(node)

            # magic_val = 2

            y = node.pos[0]
            x = node.pos[1]
            # solved = True
            # for i in range(y - magic_val, y + magic_val):
            #     for j in range(x - magic_val, x + magic_val):
            if m[y][x] != UNEXPLORED:
                return node
            else:
                children: List[Node] = node.get_children(m, explored)
                for child in children:
                    queue.append(child)
                    explored.update({child.pos: child})

        # TODO add check -1 remains

        return n
        # raise ValueError("No unexplored space found")

# magic math http

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

# nx, ny = 10, 10
# # Maze entry position
# ix, iy = 0, 0

# # allows passageways to have widths
# scaling = 8

# maze = Maze(nx, ny, scaling, ix, iy)
# maze.make_maze()


# # m is 2D matrix
# m = maze.out()
# for i in range(scaling, 2*scaling):
#     for j in range(scaling):
#         m[i][j] = UNEXPLORED


# start_pos = [floor(len(m[0])/2) + scaling, floor(len(m)/2) + scaling, 0]

# viz = Visualization(m, start_pos)

# # a = np.zeros([1, 6])
# # a = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
# a = [0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001]

# node_list: List[Node] = nearest_list(m, start_pos)

# for node in node_list:
#     print(node)

# delta_t = 1
# x_t_next = start_pos

# # while UNEXPLORED in m:
# for i in range(len(node_list) - 1):
#     motions = node_list[i].get_motion(node_list[i + 1], delta_t)

#     if type(motions[0]) == float:

#         x_t_next = Robot_motion(motions, x_t_next, a, delta_t, m, 0.01).actual_motion_model_velocity()
#         # print(x_t_next)
#         # if counter % 5 == 0:
#         viz.update(m, x_t_next, i % 40 == 0)
#     else:
#         # print(motions)
#         x_t_next = Robot_motion(motions[0], x_t_next, a, delta_t, m, 0.01).actual_motion_model_velocity()
#         x_t_next = Robot_motion(motions[1], x_t_next, a, delta_t, m, 0.01).actual_motion_model_velocity()

# viz.update(m, x_t_next)
# viz.pause()
