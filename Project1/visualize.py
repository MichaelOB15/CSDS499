from typing import List
from matplotlib import pyplot as plt, patches
from math import sin, cos
import numpy as np


class Visualization:
    """Visualization used for visualizing known map and unknown map"""

    def __init__(self, m: List[List[int]], starting_pos: List[int], r, pause=5):
        """Constructor for the Visualization Object and Draws Initial State"""
        plt.ion()
        self.m = (m-1)*-1

        # self.y_size = len(m)
        # self.x_size = len(m[0])

        self.pos = starting_pos
        self.path: List[List[int]] = [starting_pos]
        assert len(starting_pos) >= 2
        self.fig, self.ax = plt.subplots(1)
        self.r = r
        self.pause_val = pause
        self.visualize()

    def update(self, m: List[List[int]], new_pos: List[int], draw: bool = True):
        """Takes an updated position and generates a plot with the updated path built upon previous positions"""

        change_in_y = len(m) - len(self.m)
        change_in_x = len(m[0]) - len(self.m[0])

        if change_in_x > 0 or change_in_y > 0:
            self.ax.clear()

        for i in range(len(self.path)):
            self.path[i] = [self.path[i][0], self.path[i][1], self.path[i][2]]

        self.m = (m-1)*-1
        self.path.append(new_pos)
        self.pos = new_pos
        if draw:
            self.draw_movement()
            self.visualize()

    def visualize(self):
        """Helper method to plot Maze and Robot within Maze"""
        self.ax.imshow(self.m, cmap='gray', vmin=0, vmax=1)
        circle1 = patches.Circle((self.pos[1], self.pos[0]), self.r, color='blue')
        self.ax.add_patch(circle1)
        self.draw_direction()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.show(block=False)

    def draw_direction(self):
        x_0 = self.pos[1]
        y_0 = self.pos[0]
        x_1 = x_0 + cos(self.pos[2])*self.r
        y_1 = y_0 + sin(self.pos[2])*self.r
        self.ax.plot([x_0, x_1], [y_0, y_1], color="red")

    def draw_movement(self):
        """Helper method to plot path between starting pos and current pos"""
        x = []
        y = []
        for pos in self.path:
            x.append(pos[1])
            y.append(pos[0])
        self.ax.plot(x, y, color='blue')

    def pause(self):
        plt.pause(self.pause_val)
