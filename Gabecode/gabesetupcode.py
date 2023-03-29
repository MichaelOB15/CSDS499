from maze import Maze

# Number of cells in each dimension (ncols, nrows)
# each cell is a 3x3 matrix with a N,S,E,W edge
nx, ny = 5, 5
# Maze entry position (ignore this part boys)
ix, iy = 0, 0

maze = Maze(nx, ny, ix, iy)
maze.make_maze()

print(maze)

maze.write_svg('maze.svg')



