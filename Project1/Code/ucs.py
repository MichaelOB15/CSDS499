from collections import deque
from typing import List, Tuple, Any, Union
from math import floor
from visualize import Visualization

# TODO from params?
UNEXPLORED = 0.5
STEP = 1
WALL = 0
OPEN = 1


class Node:
    def __init__(self, parent, pos, cost):
        self.parent: Union[Node, None] = parent
        self.pos: Tuple[int, int] = pos
        self.cost: int = cost

    def get_children(self, m) -> List[Any]:
        children: List[Node] = []
        y = self.pos[0]
        x = self.pos[1]

        if y - 1 >= 0:
            if m[y - 1][x] == OPEN:
                children.append(Node(self, (y - 1, x), self.cost + 1))

        if y + 1 < len(m):
            if m[y + 1][x] == OPEN:
                children.append(Node(self, (y + 1, x), self.cost + 1))

        if x - 1 >= 0:
            if m[y][x - 1] == OPEN:
                children.append(Node(self, (y, x - 1), self.cost + 1))

        if x + 1 < len(m[x]):
            if m[y][x + 1] == OPEN:
                children.append(Node(self, (y, x + 1), self.cost + 1))

        print(children)
        return children

    def __str__(self) -> str:
        return f'x = {self.pos[1]}, y = {self.pos[0]}, cost = {self.cost}'


# Returns index of nearest UNEXPLORED
def nearest_unexplored(m: List[List[int]], pos: List[int]) -> Node:
    robot_x_in_m = floor(pos[0])
    robot_y_in_m = floor(pos[1])
    pos = (robot_y_in_m, robot_x_in_m)

    queue: deque[Node] = deque()
    queue.append(Node(None, pos, 0))
    while len(queue) >= 1:
        print(len(queue))
        print(queue)
        node = queue.pop()
        if m[node.pos[0]][node.pos[1]] == UNEXPLORED:
            return node
        else:
            children: List[Node] = node.get_children(m)
            for child in children:
                queue.append(child)

    # TODO add check -1 remains
    raise ValueError("No unexplored space found")


def straight_line(m: List[List[int]], pos: List[int]) -> bool:
    pass


m = [[0.5, 1, 1], [0, 0, 1], [1, 1, 1]]
start_pos = [0, 2]
# print(m)
viz = Visualization(m, start_pos)
print(nearest_unexplored(m, start_pos))

# def ucs(start_c, m):
#     map = m
#     end_c = nearest_unexplored(start_c, m)

#     # Initialize the visited array
#     visited = []
#     for i in range(0, xlen):
#         templist = []
#         for j in range(0, ylen):
#             templist.append(False)
#         visited.append(templist)

#     start = node("None", start_c, 0)
#     current = start
#     queue = deque([])
#     visited[start.pos[0]][start.pos[1]] = True
#     queue.append(start)

#     found = False

#     while queue:
#         # Pop the current node
#         current = queue.popleft()

#         # Check is current is end
#         if current.pos == end_c:
#             found = True
#             break

#         # Get the children of current
#         children = getChildren(current, visited)

#         # Sort children by cost
#         children.sort(key=sortkey)

#         # Mark as visited and add to queue
#         for i in range(0, len(children)):
#             visited[children[i].pos[0]][children[i].pos[1]] = True
#             queue.append(children[i])

#     # Step back through the map to update the shortest path
#     if found:
#         while current.parent != "None":
#             map[current.pos[0]][current.pos[1]] = '*'
#             current = current.parent
#         map[current.pos[0]][current.pos[1]] = '*'

#         # print_array(map)
#     else:
#         print("null")


# # Given a node, get its valid children and their cumilitve cost
# def getChildren(parent, tracker):
#     new_children = []
#     # Defines the range of movement
#     move = [[-1, 0],  # up
#             [1, 0],  # down
#             [0, -1],  # left
#             [0, 1]]  # right

#     for i in range(0, len(move)):
#         new_pos = move[i]

#         # Move to next position
#         next_pos = (parent.pos[0] + new_pos[0], parent.pos[1] + new_pos[1])

#         # Check for valid position
#         if next_pos[0] == -1 or next_pos[1] == -1 or next_pos[0] >= xlen or next_pos[1] >= ylen:
#             continue
#         elif map[next_pos[0]][next_pos[1]] == 'X':
#             continue
#         elif tracker[next_pos[0]][next_pos[1]]:
#             continue
#         else:
#             new_cost = int(map[next_pos[0]][next_pos[1]]) + parent.cost

#             new_node = node(parent, next_pos, new_cost)
#             new_children.append(new_node)

#     return new_children
