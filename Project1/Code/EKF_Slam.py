# 321

import numpy as np
import math

class EKF_Slam():
    def __init__(self, mew_t_1, sigma_t_1, u_t, z_t, N_t_1, alpha, simga_r, sigma_theta):
        self.mew_t_1 = mew_t_1
        self.sigma_t_1 = sigma_t_1
        self.u_t = u_t
        self.z_t = z_t
        self.N_t_1 = N_t_1
        self.alpha = alpha
        self.sigma_r = sigma_r
        self.sigma_theta = simga_theta

    def slam(self):
        
        # 2
        N_t = self.N_t_1

        # 3
        F_x = np.array([[1,0,0],[0,1,0],[0,0,1]]) + np.zeros(3,3*N)

        # 4
        theta = self.mew_t_1[2]
        mew_hat_t = self.mew_t_1 + F_x.transpose() @ np.array([-(self.u_t[0]/self.u_t[1])*math.sin(theta) + (self.u_t[0]/self.u_t[1])*math.sin(theta+self.u_t[1]*self.delta_t),
                                                     -(self.u_t[0]/self.u_t[1])*math.cos(theta) + (self.u_t[0]/self.u_t[1])*math.cos(theta+self.u_t[1]*self.delta_t),
                                                     self.u_t[1]*self.delta_t])

        # 5
        G_t = np.identity(3) + F_x.transpose() @ np.array([0,0,-(self.u_t[0]/self.u_t[1])*math.cos(theta) + (self.u_t[0]/self.u_t[1])*math.cos(theta+self.u_t[1]*self.delta_t)],
                                                          [0,0,-(self.u_t[0]/self.u_t[1])*math.sin(theta) + (self.u_t[0]/self.u_t[1])*math.sin(theta+self.u_t[1]*self.delta_t)],
                                                          [0,0,0]) @ F_x

        # 6 (TODO Still) what is R_t
        Sigma_t_bar = G_t @ self.Sigma_t_1 @  G_t.transpose() + F_x.transpose() @  

        # 7
        Q_t = np.array([[self.sigma_r**2, 0],[0, self.sigma_theta**2]])

        # 8 
        for i in range(0, len(self.z_t)):

            # 9
            mew_hat_N_1 = [mew_hat_t[0]+math.cos(self.z_t[i][1]+mew_hat_t[2]),
                           mew_hat_t[1]+math.sin(self.z_t[i][1]+mew_hat_t[2])]

            # 10
            for k in range(0, N_t+1):
                
                # 11
                delta_k = [mew_hat_t[k][0]-mew_hat_t[N_t][0], 
                           mew_hat_t[k][1]-mew_hat_t[N_t][1]]

                # 12
                q_k = delta_k.transpose() @ delta_k

                # 13
                z_hat_k_t =  [math.sqrt(q_k), math.atan2(delta_k[1], 
                              delta_k[0]) - mew_hat_t[N_t][2]]

                # 14 (TODO Finish exsisting lines)
                F_x_k = []

                # 15

                # 16

                # 17

            # 19

            # 20

            # 21

            # 22

            # 23

            # 24
        
        # 26

        # 27

        # 28
        return
