import rrt_lib
import random

def run_lazy_rrt(config, world, start_node, end_node):
    tree = [start_node]

    # check to see if start_node connects directly to end_node
    if rrt_lib.is_reached(start_node, end_node, config["step_size"], world) == True:
        path = [start_node, end_node]
    else:
        tree = extend_lazy_tree(tree, end_node, config["step_size"], world)
  
    # find path with minimum cost to end_node
    lazy_path = rrt_lib.find_min_path(tree, end_node)

    return repair_lazy_path(lazy_path, config["step_size"], world)

def repair_lazy_path(lazy_path, step_size, world):
    path = []

    is_start = False
    is_end = False

    for i in range(len(lazy_path) - 1):
        has_collision = rrt_lib.is_collided(lazy_path[i+1].pt, lazy_path[i].pt, world)
        
        if  has_collision == True and is_start == False:
            start_collision_node = lazy_path[i]
            is_start = True
            is_end = True

        elif is_end == True and has_collision == False:
            is_start = False
            is_end = False
            tree = [start_collision_node]
            end_node = lazy_path[i]
            tree = extend_tree(tree, end_node, step_size, world)

            path += rrt_lib.find_min_path(tree, end_node)
            
        elif is_start == False and is_end == False and has_collision == False:
            path.append(lazy_path[i])
            path.append(lazy_path[i + 1])

    return path

def extend_tree(tree, end_node, step_size, world: rrt_lib.World):
    while True:
        while True:
            random_pt = rrt_lib.Point(world.width * random.random(), world.length * random.random(), world.height * random.random()) 

            idx = rrt_lib.find_closet_node(random_pt, tree)
            
            new_point = random_pt - tree[idx]
            new_point = tree[idx].pt + (new_point / new_point.norm()) * step_size
            
            min_cost  = rrt_lib.cost_np(tree[idx], new_point)
            new_node  = rrt_lib.Node(new_point.x, new_point.y, new_point.z, -1, min_cost, idx)
            
            if rrt_lib.is_collided(new_node.pt, tree[idx].pt, world) == False:
                tree.append(new_node)
                break
        
        if rrt_lib.is_reached(new_node, end_node, step_size, world) == True:
            tree[-1].chi = 0
            return tree
    
def extend_lazy_tree(tree, end_node, step_size, world: rrt_lib.World):
    while True:
        random_pt = rrt_lib.Point(world.width * random.random(), world.length * random.random(), world.height * random.random())

        idx = rrt_lib.find_closet_node(random_pt, tree)
        
        new_point = random_pt - tree[idx]
        new_point = tree[idx].pt + (new_point / new_point.norm()) * step_size
        
        min_cost  = rrt_lib.cost_np(tree[idx], new_point)
        new_node  = rrt_lib.Node(new_point.x, new_point.y, new_point.z, -1, min_cost, idx)

        tree.append(new_node)

        if rrt_lib.is_reached(new_node, end_node, step_size, world) == True:
            tree[-1].chi = 0

            return tree