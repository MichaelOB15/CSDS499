
def main(config):

    # polygon generator pass in the config
        # spit out polygons with verticies and the dimmensions of the space

    # first make the obstacle

    # run the different decomposition algorithms

    # create the graphs

    # run astar on the graphs

    # analysis of all of the function

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