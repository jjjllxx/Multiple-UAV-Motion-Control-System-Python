import yaml
import rrt_lib
import traj_lib
import plot_lib
import structs
import random

def initialise_uavs(uav_num, world: structs.World):
    uavs = []
    base_width = world.width * 0.1
    base_length = world.length * 0.1
    base_height = world.height * 0.1
    
    for i in range(uav_num):
        start_pt = structs.Point(base_width * random.random(), 
                                 base_length * random.random(), 
                                 base_height * random.random())
        
        end_pt = structs.Point(base_width * (random.random() + 9), 
                               base_length * (random.random() + 9), 
                               base_height * (random.random() + 9))

        uavs.append(structs.UAV(start_pt, end_pt))

    return uavs

def main():
    with open("config.yaml", "r") as file:
        config = yaml.safe_load(file)

    world = rrt_lib.initialise_world(config)
    uavs = initialise_uavs(config["uav_num"], world)
    paths = [rrt_lib.find_path_by_rrt(uav, world, config) for uav in uavs]


    all_key_pts = [traj_lib.find_key_pts(path) for path in paths]
    trajs = [traj_lib.generate_traj(key_pts) for key_pts in all_key_pts]

    COLLISION_LIMIT = config["uav_size"] * 3
    new_trajs = traj_lib.control_multi_collision(trajs, all_key_pts, COLLISION_LIMIT)

    plot_lib.show_old_new(trajs[1], new_trajs[1])

    for path in paths:
        plot_lib.plot_result(world, path)
        
    plot_lib.plot_traj_xyz(new_trajs)
    plot_lib.save_traj_gif(world, new_trajs)

if __name__ == '__main__':
    main()