from typing import List
from matplotlib import pyplot as plt, patches
from math import sin, cos
import numpy as np


class Visualization:
    """Visualization used for visualizing known map and unknown map"""

    def __init__(self, real_m: List[List[float]], m: List[List[float]], starting_pos: List[float], r, pause=5):
        """Constructor for the Visualization Object and Draws Initial State"""
        plt.ion()
        self.m = (m-1)*-1
        self.real_m = (real_m-1)*-1
        self.real_pose = [len(real_m)/2, len(real_m[0])/2, 0]

        self.pos = starting_pos
        self.path: List[List[int]] = [starting_pos]
        assert len(starting_pos) >= 2
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2)
        self.r = r
        self.pause_val = pause
        self.visualize()

    def update(self, m: List[List[int]], new_pos: List[int], draw: bool = True):
        """Takes an updated position and generates a plot with the updated path built upon previous positions"""

        change_in_y = abs(len(m) - len(self.m))
        change_in_x = abs(len(m[0]) - len(self.m[0]))

        if change_in_x > 0 or change_in_y > 0:
            self.path = []

        self.ax1.clear()
        self.ax2.clear()

        self.m = (m-1)*-1

        y_diff = self.pos[0] - new_pos[0]
        x_diff = self.pos[1] - new_pos[1]
        theta_diff = self.pos[2] - new_pos[2]

        self.real_pose[0] = self.real_pose[0] - y_diff - change_in_y
        self.real_pose[1] = self.real_pose[1] - x_diff - change_in_x
        self.real_pose[2] = self.real_pose[2] - theta_diff

        self.path.append(new_pos)
        self.pos = new_pos
        if draw:
            self.draw_movement()
            self.visualize()

    def visualize(self):
        """Helper method to plot Maze and Robot within Maze"""
        self.ax1.imshow(self.m, cmap='gray', vmin=0, vmax=1)
        self.ax2.imshow(self.real_m, cmap='gray', vmin=0, vmax=1)

        circle1 = patches.Circle((self.pos[1], self.pos[0]), self.r, color='blue')
        circle2 = patches.Circle((self.real_pose[1], self.real_pose[0]), self.r, color='blue')

        self.ax1.add_patch(circle1)
        self.ax2.add_patch(circle2)

        self.draw_direction()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.show(block=False)

    def draw_direction(self):
        x_0 = self.pos[1]
        y_0 = self.pos[0]
        x_1 = x_0 + cos(self.pos[2])*self.r
        y_1 = y_0 + sin(self.pos[2])*self.r
        self.ax1.plot([x_0, x_1], [y_0, y_1], color="red")
        # self.ax2.plot([x_0, x_1], [y_0, y_1], color="red")

    def draw_movement(self):
        """Helper method to plot path between starting pos and current pos"""
        x = []
        y = []
        for pos in self.path:
            x.append(pos[1])
            y.append(pos[0])
        self.ax1.plot(x, y, color='blue')

    def pause(self):
        plt.pause(self.pause_val)