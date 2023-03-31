import argparse
import yaml
import os
import matplotlib.pyplot as plt
import numpy as np

from pathlib import Path
#from robot_motion import Robot_motion
#from extended_kalman_filter import EKF
#from particle_filter import PF
from math import pi, cos, sin, sqrt

from maze import Maze
from visualize import Visualization
from PIL import Image

class Main:
    def __init__(self, config, filter_type):
        self.config = config
        self.filter_type = filter_type

    def run(self):

        # Make Maze
        maze = Maze(config.maze_size[0], config.maze_size[1], config.maze_scaling, config.entry_position[0], config.entry_position[1])
        maze.make_maze()
        m = maze.out()
        save_maze_image(m)
        
        # Make Robot

        viz = Visualization(m, [883, 1224])
        viz.update(m, [883, 1224])
        viz.update(m, [883, 1224])

def save_maze_image(img):
    img = np.absolute(img-1)
    img = Image.fromarray(img.astype('uint8')*255)
    img.show()

    cdir = os.getcwd() 
    img.save(os.path.join(os.path.abspath(os.path.join(cdir, os.pardir)))+"/Maze_Image/Maze.png")

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
