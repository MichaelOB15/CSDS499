import numpy as np
from maze import Maze
from particle import Particle
from PIL import Image
from math import pi
from visualize import Visualization

from measurement_wizard import MeasurementWizard


#imports need to be fixed at the end once these files are finished


############################ MAZE CODE #############################################

nx, ny = 5, 5
ix, iy = 0, 0 # Maze entry position
scaling=30

maze = Maze(nx, ny, scaling, ix, iy)
maze.make_maze()
maze=maze.out()

############################# MAP CODE ##############################################
#book has their robot cone opening 15 degrees

a=np.array([0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001])
stepsize=1

real_pose=np.array([maze.shape[0]/2,maze.shape[1]/2,0]) #the initial position of the robot is set here
vis = Visualization(maze, real_pose)

measure=measurement_wizard()




#track real motion of the robot through the maze 
ideal=Particle()
ideal.setpose(real_pose)

u=np.array([pi/2,pi/2]) #trajectory planning will evenutally be handled by ucs.py

ideal.sample_motion_model_velocity(u,a,stepsize) #some of these parameters need to be moved to a config.yaml

#this should probably be handled inside a measure object
z=ideal_measure(ideal.getpose) #measurements are handled outside of the particle, and none of them can see the maze





#### fastSLAM occupancy grid algorithm ####

#an array of particles updates every particle with these three methods (input should be a movement u vector)

#based on the weights calculated internal to the particle the particles are resampled particle filter style

#the next trajectory is calculated here or the algorithm is ended based on there not being a suitable trajectory

#### end of fastSLAM algo ####




#a normal way to test this without trajectory code would be to just slowly spin in a circle and 
#keep the range of the sensor high enough to see all the walls...





'''
#code to generate and show a .PNG
#m = np.absolute(m-1)
#img = Image.fromarray(m.astype('uint8')*255)
#img.show()
#img.save("Maze.png")

#code to check the matrix as a .csv file
#np.savetxt("foo.csv", belief_map, delimiter=",")
'''

'''
This shows a map including the unmapped spaces
img=belief_map.copy()

for a in range(np.shape(belief_map)[0]):
    for b in range(np.shape(belief_map)[1]):
        if (img[a,b]==-1):
            img[a,b]=100
        if (img[a,b]==0):
            img[a,b]=0
        if (img[a,b]==1):
            img[a,b]=255

img = Image.fromarray(img.astype('uint8'))
img.show()
'''