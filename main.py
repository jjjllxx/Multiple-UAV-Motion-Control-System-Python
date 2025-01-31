import yaml
import rrt_lazy
import rrt_connect
import rrt_extend
import rrt_star

def main():
    with open("config.yaml", "r") as file:
        config = yaml.safe_load(file)

    if config['rrt_type'] == 'lazy':
        rrt_lazy.run_lazy_rrt(config['step_size'], config['is_random_world'])

    elif config['rrt_type'] == 'connect':
        rrt_connect.run_rrt_connect(config['step_size'], config['is_random_world'])

    elif config['rrt_type'] == 'extend':
        rrt_extend.run_rrt_extend(config['step_size'], config['is_random_world'])

    elif config['rrt_type'] == 'star':
        rrt_star.run_rrt_star(config['step_size'], config['is_random_world'], config['samples'])

    else:
        print("Invalid RRT type!")

if __name__ == '__main__':
    main()