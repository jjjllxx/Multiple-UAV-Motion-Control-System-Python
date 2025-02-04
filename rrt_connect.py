import rrt_lib
import random
import structs

def run_rrt_connect(config, world, start_node, end_node):
    tree = [start_node]

    # check to see if start_node connects directly to end_node
    if rrt_lib.is_reached(start_node, end_node, config["step_size"], world) == True:
        path = [start_node, end_node]
    else:
        is_found = False
        while not is_found:
            tree, is_found = extend_tree(tree, end_node, config["step_size"], world)

    return rrt_lib.find_min_path(tree, end_node)

def extend_tree(tree, end_node, step_size, world: structs.World):
    is_success = False

    random_pt = structs.Point(world.width * random.random(), world.length * random.random(), world.height * random.random()) 

    min_parent_idx = rrt_lib.find_closet_node(random_pt, tree)

    new_pt = tree[min_parent_idx].pt

    has_collision = False

    while (new_pt - random_pt).norm() > 0 and has_collision == False:
        if (new_pt - random_pt).norm() < step_size:
            has_collision = rrt_lib.is_collided(random_pt, tree[min_parent_idx].pt, world)
            if has_collision == False:
                
                min_cost  = rrt_lib.cost_np(tree[min_parent_idx], random_pt)
                new_node  = structs.Node(random_pt.x, random_pt.y, random_pt.z, -1, min_cost, min_parent_idx)
                tree.append(new_node)
                has_collision = True
               
                if rrt_lib.is_reached(new_node, end_node, step_size, world) == True:
                    tree[-1].chi = 0
                    is_success = True
        else:
            new_pt = random_pt - tree[min_parent_idx]
            new_pt = tree[min_parent_idx].pt + (new_pt / new_pt.norm()) * step_size
            
            min_cost  = rrt_lib.cost_np(tree[min_parent_idx], new_pt)
            new_node  = structs.Node(new_pt.x, new_pt.y, new_pt.z, -1, min_cost, min_parent_idx)
            
            has_collision = rrt_lib.is_collided(new_node.pt, tree[min_parent_idx].pt, world)
            
            if has_collision == False:
                tree.append(new_node)
                min_parent_idx = len(tree) - 1
                
                if rrt_lib.is_reached(new_node, end_node, step_size, world) == True:
                    tree[-1].chi = 0  # mark node as connecting to end.
                    has_collision = True
                    is_success = True

    return tree, is_success