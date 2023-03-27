# pg 204 - table 7.2
import numpy as np
import math

class EKF():
    def __init__(self, mew_t_1, sigma_t_1, u_t, z_t, m, delta_t, alpha, sigma_r, simga_theta):
        self.mew_t_1 = mew_t_1
        self.sigma_t_1 = sigma_t_1
        u_t[1] += 10**(-4)
        self.u_t = u_t
        self.z_t = z_t
        self.m = m
        self.delta_t = delta_t
        self.alpha = alpha
        self.sigma_r = sigma_r
        self.sigma_theta = simga_theta
        
    def filter(self):

        theta = self.mew_t_1[2]
        G_t = np.array([[1,0, - (self.u_t[0]/self.u_t[1])*math.cos(theta) + (self.u_t[0]/self.u_t[1])*math.cos(theta+self.u_t[1]*self.delta_t)],
                       [0,1, - (self.u_t[0]/self.u_t[1])*math.sin(theta) + (self.u_t[0]/self.u_t[1])*math.sin(theta+self.u_t[1]*self.delta_t)],
                       [0,0,1]])

        V_t = np.array([[(-math.sin(theta)+math.sin(theta+self.u_t[1]*self.delta_t))/self.u_t[1],self.u_t[0]*(math.sin(theta)-math.sin(theta+self.u_t[1]*self.delta_t))/(self.u_t[1]**2)+self.u_t[0]*math.cos(theta+self.u_t[1]*self.delta_t)*self.delta_t/self.u_t[1]],
                        [(-math.cos(theta)+math.cos(theta+self.u_t[1]*self.delta_t))/self.u_t[1],self.u_t[0]*(math.cos(theta)-math.cos(theta+self.u_t[1]*self.delta_t))/(self.u_t[1]**2)+self.u_t[0]*math.sin(theta+self.u_t[1]*self.delta_t)*self.delta_t/self.u_t[1]],
                        [0,self.delta_t]])

        M_t = np.array([[self.alpha[0]*(self.u_t[0]**2)+self.alpha[1]*(self.u_t[1]**2), 0],
                        [0, self.alpha[2]*(self.u_t[0]**2)+self.alpha[3]*(self.u_t[1]**2)]])

        mew_hat = np.array(self.mew_t_1) + np.array([-(self.u_t[0]/self.u_t[1])*math.sin(theta) + (self.u_t[0]/self.u_t[1])*math.sin(theta+self.u_t[1]*self.delta_t),
                                                     -(self.u_t[0]/self.u_t[1])*math.cos(theta) + (self.u_t[0]/self.u_t[1])*math.cos(theta+self.u_t[1]*self.delta_t),
                                                     self.u_t[1]*self.delta_t])

        self.sigma_t_1 = np.array(self.sigma_t_1)

        sigma_t_hat = G_t @ self.sigma_t_1 @ G_t.transpose() + V_t @ M_t @ V_t.transpose()
        
        Q_t = np.array([[self.sigma_r**2, 0],[0, self.sigma_theta**2]])

        q = (self.m[0]-mew_hat[0])**2 + (self.m[1]-mew_hat[1])**2
        
        z_t_hat = np.array([math.sqrt(q),math.atan2(self.m[1]-mew_hat[1],self.m[0]-mew_hat[0])-mew_hat[2]])

        H_t = np.array([[-(self.m[0]-mew_hat[0])/math.sqrt(q),-(self.m[1]-mew_hat[1])/math.sqrt(q),0],
                        [(self.m[1]-mew_hat[1])/q,-(self.m[0]-mew_hat[0])/q,-1]])

        S_t = H_t @ sigma_t_hat @ H_t.transpose() + Q_t

        K_t = sigma_t_hat @ H_t.transpose() @ np.linalg.inv(S_t)

        self.z_t[1] = (self.z_t[1]+2*math.pi)%(2*math.pi)
        z_t_hat[1] = (z_t_hat[1]+2*math.pi)%(2*math.pi)
        updated = (np.array(self.z_t) - z_t_hat)
        #updated[1] = ((updated[1]+4*math.pi) % (2*math.pi))

        mew_t_hat = mew_hat.reshape(1,3) + K_t @ updated

        sigma_t_hat = (np.identity(3) - K_t @ H_t) @ sigma_t_hat

        mew_t = mew_t_hat

        sigma_t = sigma_t_hat

        p_z_t = np.linalg.det(2*math.pi*S_t)**(-1/2)*math.exp((-1/2)*(z_t_hat - np.array(self.z_t).transpose()) @ np.linalg.inv(S_t) @ (z_t_hat - np.array(self.z_t).transpose()))

        return mew_t, sigma_t, p_z_t,  mew_hat, sigma_t_hat