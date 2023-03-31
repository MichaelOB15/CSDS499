import numpy as np
from maze import Maze
from PIL import Image

def resize(map):
    '''Determines whether the robot's internal map is  at risk of being too small and resizes it accordingly'''
    
    n_row=np.shape(map)[0]
    n_col=np.shape(map)[1]

    #I need to go around every edge (with a cushion) and see if I have room to work
    cushion=40
    resize_magnitude=200

    #unmapped regions will hold a value of -1
    for a in range(n_row):
        #row overflow (x axis)
        if (not map[a,n_col-cushion]==-1):
            newmap=np.zeros((n_row,resize_magnitude))-1
            map=np.concatenate([map,newmap],axis=1)
            n_col=np.shape(map)[1]
            
        #row underflow
        if (not map[a,cushion]==-1):
            newmap=np.zeros((n_row,resize_magnitude))-1
            map=np.concatenate([newmap,map],axis=1)
            n_col=np.shape(map)[1]
            
    for a in range(n_col):
        #column overflow (y axis)
        if (not map[n_row-cushion,a]==-1):
            newmap=np.zeros((resize_magnitude,n_col))-1
            map=np.concatenate([map,newmap],axis=0)
            n_row=np.shape(map)[0]
            
        #column underflow
        if (not map[cushion,a]==-1):
            newmap=np.zeros((resize_magnitude,n_col))-1
            map=np.concatenate([newmap,map],axis=0)
            n_row=np.shape(map)[0]
    
    return map

############################ MAZE CODE #############################################
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
#m = np.absolute(m-1)
#img = Image.fromarray(m.astype('uint8')*255)
#img.show()

#img.save("Maze.png")

############################# RESIZING ARRAY CODE ##############################################

#generate a resizable matrix of the map that the robot believes is the local space
#initial map size without any resizes
xdim,ydim=300,300

#initial condition
belief_map=np.zeros((xdim,ydim))-1


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

#initial pose of the robot(x,y,theta)
pose=np.zeros(3)

#x and y need to be in the center of the matrix
pose[0:1]=[xdim/2,ydim/2]




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






#a normal way to test this would be to just slowly spin in a circle and keep the range of the sensor high enough to see all the walls...





