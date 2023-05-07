from generate_workspace import Workspace
from trapezoidal_decomposition import TD
from brushfire_decomposition import Brushfire

from pathlib import Path
import yaml
import argparse


def main(config):

    map = Workspace()
    obj = map.gen()

    trap_decom = TD(obj[0], obj[1], obj[2], obj[3])
    trap_decom.vis()
    graph = trap_decom.calculate_nodes()

    graph.graph_vis()
    print(graph)
    # bf = Brushfire(obj[0],obj[1],obj[2],obj[3])

    # polygon generator pass in the config
    # spit out polygons with verticies and the dimmensions of the space

    # first make the obstacle

    # run the different decomposition algorithms

    # create the graphs

    # run astar on the graphs

    # analysis of all of the function

    ''' code for plotting the workspace, I stripped this from the method but wanted to keep because visuals/aspects were nice

        #set up plot
        plt.figure()
        plt.gca().set_aspect("equal")
        plt.xlim([-10,10])
        plt.ylim([-10,10])

        for i in range(len(scaling)):
            self.plot_polygon(output[i])


        plt.show() #plt.savefig(out_file_name, dpi=300)
        #print(polygon[0][1])
    '''
    pass


def load_config():
    config_filepath = Path.cwd() / "config.yaml"
    with config_filepath.open() as f:
        config_dict = yaml.load(f, Loader=yaml.FullLoader)
    config = argparse.Namespace()
    for key, value in config_dict.items():
        setattr(config, key, value)
    return config


if __name__ == "__main__":
    config = load_config()
    main(config)
