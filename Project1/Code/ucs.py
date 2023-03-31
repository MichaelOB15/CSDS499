# from collections import deque
from typing import List, Tuple
from math import floor
# class node:
#     def __init__(self, parent, pos, cost):
#         self.parent = parent
#         self.pos = pos
#         self.cost = cost

# TODO from params?
UNEXPLORED = -1
STEP = 1


def get_closet_unexplored(m: List[List[int]], pos: List[int]):
    pass


# Returns index of nearest UNEXPLORED
def nearest_unexplored(m: List[List[int]], pos: List[int]) -> Tuple[int, int]:
    robot_x_in_m = floor(pos[0]) 
    robot_y_in_m = floor(pos[1])

    count = 0
    while count < len(m):
        for i in range(robot_y_in_m - count, robot_y_in_m + count):
            if i < 0:
                i = 0
            elif i >= len(m):
                i = len(m) - STEP

            for j in range(robot_x_in_m - count, robot_x_in_m + count):

                if j < 0:
                    j = 0
                elif j >= len(m[0]):
                    j = len(m[0]) - STEP

                print(f'{i} , {j} = {m[i][j]}')
                if m[i][j] == UNEXPLORED:
                    return i, j
        # Expand search
        count = count + 1

    # TODO add check -1 remains
    raise ValueError("No unexplored space found")


def straight_line(m: List[List[int]], pos: List[int]) -> bool:
    pass


m = [[0, 0, 0], [-1, 0, -1], [2, 3, 4]]
print(m)
print(nearest_unexplored(m, [2, 1]))
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
