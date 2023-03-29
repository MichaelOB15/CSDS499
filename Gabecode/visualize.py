from typing import List
from matplotlib import pyplot as plt, patches
from PIL import Image
import numpy


class Visualization:
    def __init__(self, m: List[List[int]], pos: List[int]):
        self.m = m
        self.pos = pos
        self.path: List[List[int]] = [pos]
        assert len(pos) >= 2
        fig, self.ax = plt.subplots(1)
        self.visualize()

    def update(self, m: List[List[int]], new_pos: List[int]):
        fig, self.ax = plt.subplots(1)
        self.m = m
        self.path.append(new_pos)
        self.pos = new_pos
        self.draw_movement()
        self.visualize()

    def visualize(self):
        self.ax.imshow(self.m, cmap='gray', vmin=0, vmax=255)
        circle1 = patches.Circle((self.pos[0], self.pos[1]), radius=10, color='blue')
        self.ax.add_patch(circle1)
        plt.show(block=False)
        plt.pause(10)
        plt.close()

    def draw_movement(self):
        x = []
        y = []
        for pos in self.path:
            x.append(pos[0])
            y.append(pos[1])
        self.ax.plot(x, y)


pic = Image.open("Maze.png").convert("L")
M = numpy.array(pic)
test_m = [[0, 0, 0], [255, 255, 255], [0, 0, 0]]
viz = Visualization(M, [883, 1224])
viz.update(M, [883, 1224])
viz.update(M, [883, 1224])
