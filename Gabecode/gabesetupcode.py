from maze import Maze
import numpy as np
from PIL import Image

# Number of cells in each dimension (ncols, nrows)
#50 works really well
#if number is odd then center of maze will always be 0 and for other locations some scaling math has to happen
nx, ny = 50, 50
# Maze entry position
ix, iy = 0, 0

#allows passageways to have widths
scaling=30

maze = Maze(nx, ny, scaling, ix, iy)
maze.make_maze()


#m is 2D matrix
m=maze.out()
#print(m)

#I want my walls black and my passageway white
m = np.absolute(m-1)
img = Image.fromarray(m.astype('uint8')*255)
img.show()

img.save("Maze.png")