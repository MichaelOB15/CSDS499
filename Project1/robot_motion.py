# import numpy as np
# import math
# from typing import List
# # from math import floor, ceil, sqrt
# from ucs import in_wall

# SQUARE_SIZE = 1
# WALL = 1


# class Robot_motion:

#     def __init__(self, u_t, x_t_1, alpha, delta_t, maze: List[List[float]], change_in_t: float):
#         u_t[1] += 10**(-6)
#         self.u_t = u_t
#         self.x_t_1 = x_t_1
#         self.alpha = alpha
#         self.delta_t = delta_t
#         self.m = maze
#         self.change_in_t = change_in_t

#     def sample_motion_model_velocity(self):
#         while True:
#             if self.delta_t <= 0:
#                 return self.x_t_1
#             try:
#                 v_hat = self.u_t[0] + self.sample(self.alpha[0]*(self.u_t[0]**2)+self.alpha[1]*(self.u_t[1]**2))
#                 w_hat = self.u_t[1] + self.sample(self.alpha[2]*(self.u_t[0]**2)+self.alpha[3]*(self.u_t[1]**2))
#                 gamma_hat = self.sample(self.alpha[4]*(self.u_t[0]**2)+self.alpha[5]*(self.u_t[1]**2))

#                 new_x = self.x_t_1[0] - (v_hat/w_hat)*math.sin(self.x_t_1[2]) + (v_hat/w_hat)*math.sin(self.x_t_1[2]+w_hat*self.delta_t)
#                 new_y = self.x_t_1[1] + (v_hat/w_hat)*math.cos(self.x_t_1[2]) - (v_hat/w_hat)*math.cos(self.x_t_1[2]+w_hat*self.delta_t)
#                 new_theta = self.x_t_1[2] + w_hat*self.delta_t + gamma_hat*self.delta_t

#                 pos = [round(new_x, 3), round(new_y, 3), round(new_theta, 3)]
#                 if not in_wall(self.m, pos):
#                     return pos
#                 else:
#                     self.delta_t = self.delta_t - self.change_in_t
#             except Exception:
#                 print(self.u_t)
#                 print(self.x_t_1)

#     def actual_motion_model_velocity(self):
#         while True:
#             if self.delta_t <= 0:
#                 return self.x_t_1

#             new_x = self.x_t_1[0] - (self.u_t[0]/self.u_t[1])*math.sin(self.x_t_1[2]) + (self.u_t[0]/self.u_t[1])*math.sin(self.x_t_1[2]+self.u_t[1]*self.delta_t)
#             new_y = self.x_t_1[1] + (self.u_t[0]/self.u_t[1])*math.cos(self.x_t_1[2]) - (self.u_t[0]/self.u_t[1])*math.cos(self.x_t_1[2]+self.u_t[1]*self.delta_t)
#             new_theta = self.x_t_1[2] + self.u_t[1]*self.delta_t

#             pos = [round(new_x, 3), round(new_y, 3), round(new_theta, 3)]
#             if not in_wall(self.m, pos):
#                 return pos
#             else:
#                 self.delta_t = self.delta_t - self.change_in_t

#     def sample(self, b):
#         return np.random.normal(0, b)
