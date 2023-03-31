import numpy as np
from maze import Maze
from particle import Particle
from PIL import Image
from math import pi
from visualize import Visualization


############################ MAZE CODE #############################################
# Number of cells in each dimension (ncols, nrows)
#50 works really well
#if number is odd then center of maze will always be 0 and for other locations some scaling math has to happen
nx, ny = 5, 5
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
#m = np.absolute(m-1)
#img = Image.fromarray(m.astype('uint8')*255)
#img.show()

#img.save("Maze.png")



'''
This was super useful for testing! I want to refer to it later
#update with data
belief_map[20:ydim-20,20:xdim-20]=m[20:ydim-20,20:xdim-20]

img=belief_map.copy()

for a in range(np.shape(belief_map)[0]):
    for b in range(np.shape(belief_map)[1]):
        if (img[a,b]==-1):
            img[a,b]=100
        if (img[a,b]==0):
            img[a,b]=0
        if (img[a,b]==1):
            img[a,b]=255

#delete me!
#np.savetxt("foo.csv", belief_map, delimiter=",")

img = Image.fromarray(img.astype('uint8'))
img.show()

#resize

belief_map=resize(belief_map)

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

############################# MAP CODE ##############################################
#book has their robot cone opening 15 degrees
#based on pg 303 it's not unusual to have a robot with evenly spaced sensors on all sides
#I really want the output of this whole project to look like pg 303 so thats maybe the way to do it?
#nah lets do this the budget way with one (maybe two) really dumb sensors on the front
#u=np.array([0,0])

'''
a=np.array([0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001])
stepsize=1
p=Particle()
p.setpose(np.array([2,0,pi/2]))
u=np.array([pi/2,pi/2])
p.sample_motion_model_velocity(u,a,stepsize)
u=np.array([pi/2,-pi/2])
p.sample_motion_model_velocity(u,a,stepsize)
'''

#inside my script here I will take measurements:
real_pose=np.array([m.shape[0]/2,m.shape[1]/2,0]) #the initial position of the robot is set here
vis = Visualization(m, real_pose)
#print(m[m.shape[0]/2,m.shape[1]/2].astype(int))
mrow = (m.shape[0]/2)
mcol = (m.shape[1]/2)
print(mrow)
print(mcol)
print(m[int(mrow),int(mcol)])


#I'm going to write a particle class -> each particle is a representation of the robot

#### class particle #####

#methods:
#-particle moves with velocity model plus some error (sample velocity model)
#-particle takes a measurement with all sensors and back-calculates likelihood of position (measurement model map)
#-based on measurements particle updates its own local map accordingly

#ORDER MATTERS so each method should probably proc the next method
#use pg 478 as a reference

#### end class particle ####    #I've moved this comment into its own file titled particle.py




#### fastSLAM occupancy grid algorithm ####

#an array of particles updates every particle with these three methods (input should be a movement u vector)

#based on the weights calculated internal to the particle the particles are resampled particle filter style

#the next trajectory is calculated here or the algorithm is ended based on there not being a suitable trajectory

#### end of fastSLAM algo ####






#a normal way to test this without trajectory code would be to just slowly spin in a circle and 
#keep the range of the sensor high enough to see all the walls...





