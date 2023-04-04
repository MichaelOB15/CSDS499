from typing import List
from matplotlib import pyplot as plt, patches
from math import sin, cos
import numpy as np


# RADIUS = 3


class Visualization:
    """Visualization used for visualizing known map and unknown map"""

    def __init__(self, m: List[List[int]], starting_pos: List[int], r, pause = 5):
        """Constructor for the Visualization Object and Draws Initial State"""
        plt.ion()
        self.m = np.abs(m-1)
        self.pos = starting_pos
        self.path: List[List[int]] = [starting_pos]
        assert len(starting_pos) >= 2
        self.fig, self.ax = plt.subplots(1)
        self.r = r
        self.pause = 5
        self.visualize()

    def update(self, m: List[List[int]], new_pos: List[int], draw: bool = True):
        """Takes an updated position and generates a plot with the updated path built upon previous positions"""
        self.m = np.abs(m-1)
        self.path.append(new_pos)
        self.pos = new_pos
        if draw:
            self.draw_movement()
            self.visualize()

    def visualize(self):
        """Helper method to plot Maze and Robot within Maze"""
        self.ax.imshow(self.m, cmap='gray', vmin=0, vmax=1)
        circle1 = patches.Circle((self.pos[0], self.pos[1]), self.r, color='blue')
        self.ax.add_patch(circle1)
        self.draw_direction()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.show(block=False)

    def draw_direction(self):
        x_0 = self.pos[0]
        y_0 = self.pos[1]
        x_1 = x_0 + cos(self.pos[2])*self.r
        y_1 = y_0 + sin(self.pos[2])*self.r
        self.ax.plot([x_0, x_1], [y_0, y_1], color="red")

    def draw_movement(self):
        """Helper method to plot path between starting pos and current pos"""
        x = []
        y = []
        for pos in self.path:
            x.append(pos[0])
            y.append(pos[1])
        self.ax.plot(x, y, color='blue')

    def pause(self):
        plt.pause(self.pause)
