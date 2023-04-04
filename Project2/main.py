import numpy as np
from maze import Maze
from particle import Particle
from math import pi
import random
from visualize import Visualization
from measurement_wizard import MeasurementWizard
from ucs import nearest_list

from PIL import Image


# imports need to be fixed at the end once these files are finished
# we should probably go through this at the end and fix up my weird naming conventions etc idk what the standard is

############################ MAZE CODE #############################################

nx, ny = 2, 2
ix, iy = 0, 0  # Maze entry position
scaling = 6

maze = Maze(nx, ny, scaling, ix, iy)
maze.make_maze()
maze = maze.out()

############################# MEASUREMENT CODE ##############################################


# this is extra and shouldnt be done until the very end but there's
# maybe a way to visualize where we just track the measure particle
# I cant use the visualize class as I go cuz it's really slow :(


stepsize = 5
real_pose = np.array([maze.shape[0]/2, maze.shape[1]/2, 0])  # the initial position of the robot is set here
measure = MeasurementWizard(maze, real_pose)



delta_t = 1

# initialize an array of particles
num_particles=100
particle_samples=np.empty(num_particles,dtype=Particle)
for n in range(num_particles):
    particle_samples[n]=Particle()

# print(particle_samples[0].get_map())
# vis = Visualization(particle_samples[0].get_map(), real_pose)

'''
while 0.5 in particle_samples[0].get_map():
    l = nearest_list(particle_samples[0].get_map(), particle_samples[0].get_pose())
    for i in range(len(l) - 1):
        motions = l[i].get_motion(l[i + 1], delta_t)
        if type(motions[0]) == float:
            recieve_motion_command(motions)
            # vis.update(particle_samples[0].get_map(), particle_samples[0].get_pose(), i % 40 == 0)
            # vis.pause()
        else:
            recieve_motion_command(motions[0])
            recieve_motion_command(motions[1])

print("Mapped Maze!")
'''

def recieve_motion_command(u,particle_samples):

    #move measurement wizard according to command
    z=measure.navigate_maze(u,stepsize)

    #move particles according to command
    for i in range(num_particles):
        particle_samples[i].sample_motion_model_velocity(u,stepsize) #should probably put stepsize in config
        particle_samples[i].set_measurements(z) #recieves measurement
        particle_samples[i].likelihood_field_range_finder_model() #measurement model

    #rejection sampling to see which robots survive -> this converges faster if I narrow down the range of my guesses
    maxweight=0
    for i in range(num_particles):
        if particle_samples[i].getweight()>maxweight:
            maxweight=particle_samples[i].getweight()

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
            new_samples[i].update_occupancy_grid_map()
            weightlog[i]=new_samples[i].getweight()

    #now I need to np.copy() the map in each particle to split the objects and make them independent
    for i in range(num_particles):
        new_samples[i].setmap(new_samples[i].getmap().copy())

    #overwrite the old set of samples
    return new_samples 
    


u=np.array([1,0])

particle_samples = recieve_motion_command(u,particle_samples)

img=particle_samples[5].get_map().copy()

#images have high intensity as white so walls need to change to zeros
for a in range(np.shape(img)[0]):
    for b in range(np.shape(img)[1]):
        if (img[a,b]==-1):
            img[a,b]=100
        if (img[a,b]==0):
            img[a,b]=0
        if (img[a,b]==1):
            img[a,b]=255

img = Image.fromarray(img.astype('uint8'))
img.show()


    












# the next trajectory is calculated here or the algorithm is ended based on there not being a suitable trajectory



#visualization stuff here

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