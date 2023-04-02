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

############################# MEASUREMENT CODE ##############################################

#vis = Visualization(maze, real_pose)
#this is extra and shouldnt be done until the very end but there's 
#maybe a way to visualize where we just track the measure particle
#I cant use the visualize class as I go cuz it's really slow :(


a=np.array([0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001])
stepsize=5
real_pose=np.array([maze.shape[0]/2,maze.shape[1]/2,0]) #the initial position of the robot is set here
u=np.array([1,0]) #trajectory planning will evenutally be handled by ucs.py



measure = MeasurementWizard(maze, real_pose)
z=measure.navigate_maze(u,a,stepsize)
#z=measure.getZ() #also works

print(z)


#this is a mess... I know the ideal_measure code works so i'll rework how these fit together

'''
leave this here in case i need to make more sensor measurement photos?
print(measure.ideal_measure(rmax,dr,dtheta))
img = maze.copy()
img[int(measure.getpose()[0]),int(measure.getpose()[1])]=1
measure.navigate_maze(u,a,stepsize)


print(measure.ideal_measure(rmax,dr,dtheta))
img=img+maze.copy()
img[int(measure.getpose()[0]),int(measure.getpose()[1])]=1
measure.navigate_maze(u,a,stepsize)
'''

'''
print(measure.ideal_measure(rmax,dr,dtheta))
img=img+maze.copy()
img[int(measure.getpose()[0]),int(measure.getpose()[1])]=1
measure.navigate_maze(u,a,stepsize)

print(measure.ideal_measure(rmax,dr,dtheta))
img=img+maze.copy()
img[int(measure.getpose()[0]),int(measure.getpose()[1])]=1
'''

'''
#images have high intensity as white so walls need to change to zeros
for a in range(np.shape(img)[0]):
    for b in range(np.shape(img)[1]):
        if (img[a,b]<0):
            img[a,b]=100
        if (img[a,b]==0):
            img[a,b]=255
        if (img[a,b]==1):
            img[a,b]=0

img = Image.fromarray(img.astype('uint8'))
img.show()
img.save("MazeTestingRangefinderFunction.png")

#z=measure.navigate_maze(u) #spits out the measurements

'''

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