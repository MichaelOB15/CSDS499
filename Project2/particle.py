import numpy as np
from math import sin, cos, sqrt


'''
    things to put in config.yaml:
    initial map sizes for the particle
    a
    stepsize
    offset
    
'''

class Particle:
    def __init__(self):
        """Initialize the particle at the center of its internal map."""

        #initial map size without any resizes
        row,col=300,300

        #initial condition
        self.map=np.zeros((row,col))-1
        self.pose=np.array([row/2,col/2,0]) #y,x,theta
        self.weight=None
        self.measurements=None


    #there should be a particle.update(u) that runs the three update commands... ORDER MATTERS. implement this last

    #use pg 478 as a reference for an overview of the full algorithm


    def setpose(self,pose):
        #pose has the potential to leave the map
        map=self.map

        #map will be expanded by the difference between the default pose plus a cushion
        cushion=50 #note that this method means repeated setpose() calls will repeatedly upsize the map
        
        row_error=self.pose[0]-pose[0] +cushion #instead of having the cushion be the size of the map use values from .yaml file
        col_error=self.pose[1]-pose[1] +cushion
        
        #value needs to be an integer greater than zero
        if row_error<1:
            row_error=1
        if col_error<1:
            col_error=1
            
        n_col=np.shape(map)[1]
        
        #row overflow code seen in resize()
        newmap=np.zeros((int(row_error),n_col))-1
        map=np.concatenate([map,newmap],axis=0)

        n_row=np.shape(map)[0]

        #column overflow code seen in resize()
        newmap=np.zeros((n_row,int(col_error)))-1
        map=np.concatenate([map,newmap],axis=1)

        self.map=map
        self.pose=pose

    def getpose(self):
        return self.pose

    def sample_motion_model_velocity(self,u0,a,stepsize):
        '''Move the location of the robot with trajectory error'''

        v=u0[0]
        w=u0[1]

        variance = np.zeros(3)
        variance[0]=a[0]*abs(v)+a[1]*abs(w)
        variance[1]=a[2]*abs(v)+a[3]*abs(w)
        variance[2]=a[4]*abs(v)+a[5]*abs(w)

        offset=1e-18

        #avoids any divide by zero errors
        variance = variance + offset

        #sample normal distribution:
        mu=0
        sigma=np.zeros(3)
        for i in range(len(variance)):
            sigma[i]=sqrt(variance[i])
        epsilon=np.random.normal(mu,sigma)

        #add error to motion command
        u=u0+epsilon[0:1]

        #reset motion, this time with error
        v=u[0]
        w=u[1]
        theta=self.pose[2]

        #apply motion
        x_update=-v/w*sin(theta)+v/w*sin(theta+w*stepsize)
        y_update=v/w*cos(theta)-v/w*cos(theta+w*stepsize)
        theta_update=(w+epsilon[2])*stepsize

        self.pose=self.pose+np.array([y_update,x_update,theta_update])
        print(self.pose)

   
    def measurement_model_map(self,map,z): #z in this method comes from measurement.py and is sent in from the main.py script
        '''Set the weight of the particle'''

        #algorithm on pg 288

        #I think the algo for the next method is on pg 286
        pass

    def updated_occupancy_grid():
        '''update the map based on measurement data'''
        

        pass
        

    
    #####particle takes a measurement with all sensors and back-calculates likelihood of position (measurement model map)
    #here is where we write the sensor model -> use several spaced out ultrasound sensors, and these sensors are described as:
    # the correct algorithm for range finder sensor modelling is on book pg 172 (this is too simple)
    # localization using an ultrasound sensor is on pg 288 -> inverse problem (I think this is the right one)
    # I think we want a combo of pg 301 algorithm plus the dummy pg 172 one for just the mapping... (this is wrong but I dont want to lose page)
    #book has their robot cone opening 15 degrees
    #based on pg 303 it's not unusual to have a robot with evenly spaced sensors on all sides

    def resize(self,map):
        '''Determines whether the robot's internal map is  at risk of being too small and resizes it accordingly'''
        
        n_row=np.shape(self.map)[0]
        n_col=np.shape(self.map)[1]

        #I need to go around every edge (with a cushion) and see if I have room to work
        cushion=40 ########################################if these two are added to .yaml file then setpose method also could use these 
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

                #underflows need to update pose
                self.pose[1]=self.pose[1]+resize_magnitude
                
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

                #underflows need to update pose
                self.pose[0]=self.pose[0]+resize_magnitude
        
        return map
    

    