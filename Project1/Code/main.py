import argparse
import yaml
import os
import matplotlib.pyplot as plt
import numpy as np

from pathlib import Path
from robot_motion import Robot_motion
from extended_kalman_filter import EKF
from particle_filter import PF
from ellipse import Ellipse
from math import pi, cos, sin, sqrt

class Main:
    def __init__(self, config, filter_type):
        self.config = config
        self.filter_type = filter_type

    def run(self):
        pass

        # Gabe put new setup code here

        '''
        positions = [config.x_0]
        robot_pos = config.x_0
        print("Starting Robot Position:",robot_pos)

        EKF_Readings = []
        EKF_Readings.append([[config.x_0, config.initial_sigma_t_1, 1.0, config.initial_sigma_t_1]])

        non_adjusted_ellipse_readings = []

        adjusted_ellipse_readings = []

        adjusted_Particle_Filter_Readings = []
        non_adjusted_Particle_Filter_Readings = []

        X_t_1 = []
        X_t_1_bar = []
        for i in range(0,config.num_particles):
            X_t_1.append([2,2,0])
            X_t_1_bar.append([[2,2,0], 0])
        adjusted_Particle_Filter_Readings.append(X_t_1)
        non_adjusted_Particle_Filter_Readings.append(X_t_1_bar)


        for i in range(0, len(config.motion_commands)):
            print("Movement (",i+1,")")
            robot = Robot_motion(config.motion_commands[i], robot_pos, config.alpha, config.delta_t)
            robot_pos = robot.actual_motion_model_velocity()
            #print(robot_pos)
            
            positions.append(robot_pos)

            #print("Actual Robot position: ", robot_pos)
            if i == 0:
                sigma_t = config.initial_sigma_t_1
                mew_t = [config.x_0]

            # EKF Filter
            filter = EKF(mew_t[0], 
                        sigma_t, 
                        config.motion_commands[i], 
                        config.measurements[i][0:2], 
                        config.marker_locations[config.measurements[i][2]-1], 
                        config.delta_t, 
                        config.alpha, 
                        config.sigma_r, 
                        config.sigma_theta)

            mew_t, sigma_t, p_z_t, mew_hat, sigma_t_hat = filter.filter()
            EKF_Readings.append([mew_t, sigma_t, p_z_t, mew_hat, sigma_t_hat])

            ellipse = Ellipse(sigma_t[0:2, 0:2])
            adjusted_ellipse_readings.append(ellipse.error_to_ellipse())

            ellipse = Ellipse(sigma_t_hat[0:2, 0:2])
            non_adjusted_ellipse_readings.append(ellipse.error_to_ellipse())
            
            # Particle Filter
            particle_filter = PF(adjusted_Particle_Filter_Readings[-1], 
                                config.motion_commands[i], 
                                config.measurements[i], 
                                config.marker_locations[config.measurements[i][2]-1], 
                                config.alpha, 
                                config.delta_t, 
                                config.sigma_r, 
                                config.sigma_theta)

            X_t, X_bar_t = particle_filter.filter()

            adjusted_Particle_Filter_Readings.append(X_t)
            non_adjusted_Particle_Filter_Readings.append(X_bar_t)

        # Plot Particle Filter
        print("Plotting Particle")
        plt.figure(1)

        for i in range(0, len(adjusted_Particle_Filter_Readings)):
            print("Iteration:",i)
            for k in range(0, len(adjusted_Particle_Filter_Readings[i])):
                if i+1 == len(adjusted_Particle_Filter_Readings) and k+1 == len(adjusted_Particle_Filter_Readings[i]):
                    plt.scatter(adjusted_Particle_Filter_Readings[i][k][0], adjusted_Particle_Filter_Readings[i][k][1], s=1, color="blue", label="Adjusted")
                    plt.scatter(non_adjusted_Particle_Filter_Readings[i][k][0][0], non_adjusted_Particle_Filter_Readings[i][k][0][1], s=1, color="black", label="Not Adjusted")
                else:
                    plt.scatter(adjusted_Particle_Filter_Readings[i][k][0], adjusted_Particle_Filter_Readings[i][k][1], s=1, color="blue")
                    plt.scatter(non_adjusted_Particle_Filter_Readings[i][k][0][0], non_adjusted_Particle_Filter_Readings[i][k][0][1], s=1, color="black")

        plt.legend(loc="upper right")

        cdir = os.getcwd() 
        plt.savefig(os.path.join(os.path.abspath(os.path.join(cdir, os.pardir)))+ "/Images/ParticleFilter.png", dpi = 72)

        # Plot EKF Filter
        print("Plotting EKF")
        plt.figure(2)
        x_points = []
        y_points = []
        actual_x = []
        actual_y = []
        for i in range(0, len(EKF_Readings)):
            x_points.append(EKF_Readings[i][0][0][0])
            y_points.append(EKF_Readings[i][0][0][1])
            actual_x.append(positions[i][0])
            actual_y.append(positions[i][1])

            # Plot ellipse
            if i != 0:
                
                # non Adjust eclipsee
                u = EKF_Readings[i][3][0]
                v = EKF_Readings[i][3][1]
                a, b, t_rot = non_adjusted_ellipse_readings[i-1]
                a = sqrt(a)
                b = sqrt(b)

                t = np.linspace(0, 2*pi, 100)
                Ell = np.array([a*np.cos(t) , b*np.sin(t)])  
                R_rot = np.array([[cos(t_rot) , -sin(t_rot)],[sin(t_rot) , cos(t_rot)]])  

                Ell_rot = np.zeros((2,Ell.shape[1]))
                for k in range(Ell.shape[1]):
                    Ell_rot[:,k] = np.dot(R_rot,Ell[:,k])

                if i+1 == len(EKF_Readings): 
                    plt.plot( u+Ell_rot[0,:] , v+Ell_rot[1,:], color = 'darkblue', label = "Initial Error")
                else:
                    plt.plot( u+Ell_rot[0,:] , v+Ell_rot[1,:], color = 'darkblue')    #rotated ellipse


                # Adjusted eclipse
                u = EKF_Readings[i][0][0][0]
                v = EKF_Readings[i][0][0][1]
                a, b, t_rot = adjusted_ellipse_readings[i-1]
                a = sqrt(a)
                b = sqrt(b)

                t = np.linspace(0, 2*pi, 100)
                Ell = np.array([a*np.cos(t) , b*np.sin(t)])  
                R_rot = np.array([[cos(t_rot) , -sin(t_rot)],[sin(t_rot) , cos(t_rot)]])  

                Ell_rot = np.zeros((2,Ell.shape[1]))
                for k in range(Ell.shape[1]):
                    Ell_rot[:,k] = np.dot(R_rot,Ell[:,k])

                if i+1 == len(EKF_Readings):
                    plt.plot( u+Ell_rot[0,:] , v+Ell_rot[1,:], color = 'darkorange', label = "Adjusted Error")
                else:  
                    plt.plot( u+Ell_rot[0,:] , v+Ell_rot[1,:], color = 'darkorange')

        # Add markers to the graph

        plt.legend(loc="upper right")
        plt.title("EKF")
        plt.plot(actual_x, actual_y, color='green', label = "Ideal Path")
        plt.plot(x_points, y_points, color='red', label="Actual Path")
        plt.scatter(actual_x, actual_y, color='black')
        plt.scatter(x_points,y_points, color='black')

        cdir = os.getcwd() 
        plt.savefig(os.path.join(os.path.abspath(os.path.join(cdir, os.pardir)))+ "/Images/EKF.png", dpi = 72)
        plt.show()'''

def load_config():
    config_filepath = Path.cwd() / "config.yaml"
    with config_filepath.open() as f:
        config_dict = yaml.load(f, Loader=yaml.FullLoader)
    config = argparse.Namespace()
    for key, value in config_dict.items():
        setattr(config, key, value)
    return config

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Can give either robot filter as arguments
    # Refrence NLP Argparser in project 1
    args = parser.parse_args()
    config = load_config()

    main = Main(config,"filter")
    main.run()
