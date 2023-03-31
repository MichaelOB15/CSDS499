#I'm going to write a particle class -> each particle is a representation of the robot

#### class particle #####
#particle has local map initialized -> robot placed perfectly in the center of local matrix but no data
#look at resize method in gabecode.py and run this resize method in the particle. tbh this method should just be moved to this class...


#methods:
#####particle moves with velocity model plus some error (sample velocity model)


#####particle takes a measurement with all sensors and back-calculates likelihood of position (measurement model map)
#here is where we write the sensor model -> use several spaced out ultrasound sensors, and these sensors are described as:
# the correct algorithm for range finder sensor modelling is on book pg 172 (this is too simple)
# localization using an ultrasound sensor is on pg 288 -> inverse problem (I think this is the right one)
# I think we want a combo of pg 301 algorithm plus the dummy pg 172 one for just the mapping... (this is wrong but I dont want to lose page)
#book has their robot cone opening 15 degrees
#based on pg 303 it's not unusual to have a robot with evenly spaced sensors on all sides


#####based on measurements particle updates its own local map accordingly


#ORDER MATTERS so each method should probably proc the next method
#use pg 478 as a reference for an overview of the full algorithm

#### end class particle ####





#Third, I force robot to go through certain poses and make sure that the robot local map updates properly