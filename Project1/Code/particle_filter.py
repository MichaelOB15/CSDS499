# PG 98 4.3 Particle Filter
# PG 252 8.2 Monte Carlo Localization
# "draw i with probability"
import numpy as np
from robot_motion import Robot_motion
from known_correspondance import LMKC
import random


class PF():
    # how do we determine the number of particles

    def __init__(self, X_t_1, u_t, z_t, m, alpha, delta_t, sigma_r, sigma_theta):
        self.X_t_1 = X_t_1
        self.u_t = u_t
        self.z_t = z_t
        self.m = m
        self.alpha = alpha
        self.delta_t = delta_t
        self.sigma_r = sigma_r
        self.sigma_theta = sigma_theta

    def filter(self):
        X_bar_t = []
        X_t = []
        max_w = 0
        for i in range(0, len(self.X_t_1)):
            samp = Robot_motion(self.u_t, self.X_t_1[i], self.alpha, self.delta_t)
            x_t = samp.sample_motion_model_velocity()

            measurement = LMKC(self.z_t, x_t, self.m, self.sigma_r, self.sigma_theta)
            w_t = measurement.landmark_model_known_correspondance()

            if w_t >max_w:
                max_w = w_t

            X_bar_t.append([x_t,w_t])

        for i in range(0, len(X_bar_t)):
            a = 0

            while a == 0:
                x = random.randint(0,len(X_bar_t)-1)

                h1 = X_bar_t[x][1]
                h2 = max_w * random.random()

                if h2 <= h1:
                    X_t.append(X_bar_t[x][0])
                    a = 1

        return X_t, X_bar_t

'''
X_t_1 = []
for i in range(0,1000):
    X_t_1.append([2,2,0])
u_t = [1,0]
z_t = [2.276,5.249]
m = [4.0,0.0]
alpha = [0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001]
delta_t = 1
sigma_r = .1
sigma_theta = .09

particle_filter = PF(X_t_1, u_t, z_t, m, alpha, delta_t, sigma_r, sigma_theta)
print(particle_filter.filter())'''