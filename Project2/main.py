import numpy as np
from maze import Maze
from particle import Particle
from math import pi
import random
from visualize import Visualization
from measurement_wizard import MeasurementWizard


# imports need to be fixed at the end once these files are finished
# we should probably go through this at the end and fix up my weird naming conventions etc idk what the standard is

############################ MAZE CODE #############################################

nx, ny = 5, 5
ix, iy = 0, 0  # Maze entry position
scaling = 30

maze = Maze(nx, ny, scaling, ix, iy)
maze.make_maze()
maze = maze.out()

############################# MEASUREMENT CODE ##############################################

# vis = Visualization(maze, real_pose)
# this is extra and shouldnt be done until the very end but there's
# maybe a way to visualize where we just track the measure particle
# I cant use the visualize class as I go cuz it's really slow :(


stepsize = 5
real_pose = np.array([maze.shape[0]/2, maze.shape[1]/2, 0])  # the initial position of the robot is set here
measure = MeasurementWizard(maze, real_pose)

#initialize an array of particles
num_particles=100
particle_samples=np.empty(num_particles,dtype=Particle)
for n in range(num_particles):
    particle_samples[n]=Particle()


def recieve_motion_command(u):

    #move measurement wizard according to command
    measure.navigate_maze(u,stepsize)
    

    #move particles according to command
    for i in range(num_particles):
        particle_samples[n].sample_motion_model_velocity(u,stepsize) #should probably put stepsize in config
        #give robot measurement and have it establish weights
        #particle_samples[n].sample_motion_model_velocity(u,stepsize) #should probably put stepsize in config


    #rejection sampling to see which robots survive -> this converges faster if I narrow down the range of my guesses
    maxweight=0
    for i in range(num_particles):
        if particle_samples[n].getweight()>maxweight:
            maxweight=particle_samples[n].getweight()

    #implement rejection sampling
    new_samples=np.empty(num_particles,dtype=Particle)
    for i in range(num_particles):
        j=0
        while j==0:
            samplenumber=random.randint(0,num_particles)

            a=particle_samples[samplenumber].getweight()
            b=random.random()*maxweight*1.1 #scaled up so the weight guess is solidly above the largest weight

            if b<=a:
                j=1
                new_samples[i]=particle_samples[samplenumber] #I want to pass the address in memory not split the object

    #update map, but update is expensive so only run if particle not seen before
    weightlog=np.zeros(num_particles)
    for i in range(num_particles):
        if not new_samples[i].getweight().isin(weightlog):
            #new_samples[i].(some code here that updates the map it's not written properly yet)
            weightlog[i]=new_samples[i].getweight()

    #now I need to np.copy() the map in each particle to split the objects and make them independent

    particle_samples=new_samples



        


    












# the next trajectory is calculated here or the algorithm is ended based on there not being a suitable trajectory

#### end of fastSLAM algo ####




# a normal way to test this without trajectory code would be to just slowly spin in a circle and
# keep the range of the sensor high enough to see all the walls...



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

#images have high intensity as white so walls need to change to zeros
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