from generate_workspace import Workspace
from trapezoidal_decomposition import TD
from brushfire_decomposition import Brushfire
from point import Point, EUCLIDEAN

from pathlib import Path
import yaml
import argparse


def main(config):

    map = Workspace()
    obj = map.gen()

    trap_decom = TD(obj[0], obj[1], obj[2], obj[3])
    graph = trap_decom.calculate_nodes()
    trap_decom.vis()

    graph.graph_vis()
    start = Point(obj[2][0], obj[2][1])
    end = Point(obj[3][0], obj[3][1])
    graph.path_graph(start, end, EUCLIDEAN)

    # bf = Brushfire(obj[0],obj[1],obj[2],obj[3])
    # img=bf.brushfireAlg()

    # bf.wavefront()
    # val=bf.get_map()

    # [ind1,ind2]=[len(val),len(val[0])]
    # #img=np.zeros([ind1,ind2])
    # for x in range(ind1):
    #     for y in range(ind2):
    #         img[x,y]=val[x][y]/4-img[x,y]

    # print(img[obj[2][0],obj[2][1]])

    # nodes=[]
    # nodes.append(obj[2])
    # while(not nodes[-1]==obj[3]):
    #     x=nodes[-1][0]
    #     y=nodes[-1][1]

    #     min=float('inf')
    #     coord=[0,0]

    #     for a in range(3):
    #         for b in range(3):
    #             if(img[a,b]<min):
    #                 min=img[a,b]
    #                 coord=[a,b]
    #     print(coord)
    #     nodes.append(coord)
    #     print(nodes)

    #     #set up plot
    #     plt.figure()
    #     plt.gca().set_aspect("equal")
    #     plt.xlim([-10,10])
    #     plt.ylim([-10,10])

    #     x = []
    #     y = []
    #     for i in range(len(nodes)):
    #         x.append(nodes[i][0])
    #         y.append(nodes[i][1])
    #     plt.plot(x, y)
    #     # plt.show()
    #         # self.plot_polygon(output[i])


    # plt.show() #plt.savefig(out_file_name, dpi=300)
    # print(polygon[0][1])

    #img = Image.fromarray(img.astype('uint8')*5)
    #img.show()
    #img.save("brushfire.png")
    # img = Image.fromarray(img.astype('uint8'))
    # img.show()
    # img.save("brushfire.png")
    # polygon generator pass in the config
    # spit out polygons with vertices and the dimmensions of the space

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
