import yaml
import lazy_rrt


def main():
    with open("config.yaml", "r") as file:
        config = yaml.safe_load(file)

    if config['rrt_type'] == 'lazy':
        lazy_rrt.run_lazy_rrt(config['step_size'], config['is_random_world'])

    elif config['rrt_type'] == 'connect':
        pass
    elif config['rrt_type'] == 'extend':
        pass
    elif config['rrt_type'] == 'star':
        pass
    else:
        print("Invalid RRT type!")

if __name__ == '__main__':
    main()