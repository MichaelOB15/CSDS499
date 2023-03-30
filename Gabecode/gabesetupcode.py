from maze import Maze
from visualize import Visualization
import numpy as np
from PIL import Image

# Number of cells in each dimension (ncols, nrows)
# 50 works really well
# if number is odd then center of maze will always be 0 and for other locations some scaling math has to happen
nx, ny = 50, 50
# Maze entry position
ix, iy = 0, 0

scaling = 30
maze = Maze(nx, ny, scaling, ix, iy)
maze.make_maze()


# m is 2D matrix
m = maze.out()

# I want my walls black and my passageway white
img = m
img = np.absolute(img-1)

img = Image.fromarray(img.astype('uint8')*255)
img.show()
img.save("Maze.png")

m = m
# I cant figure out the segmentation fault here for the life of
viz = Visualization(m, [883, 1224])
viz.update(m, [883, 1224])
viz.update(m, [883, 1224])
