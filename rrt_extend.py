import rrt_lib
import random
import structs

def run_rrt_extend(config, world, start_node, end_node):
    tree = [start_node]

    if rrt_lib.is_reached(start_node, end_node, config["step_size"], world) == True:
        path = [start_node, end_node]
    else:
        is_found = False
        while not is_found:
            tree, is_found = extend_tree(tree, end_node, config["step_size"], world)

    return rrt_lib.find_min_path(tree, end_node)

def extend_tree(tree, end_node, step_size, world):
    while True:
        random_pt = structs.Point(world.width * random.random(), world.length * random.random(), world.height * random.random()) 
        
        min_parent_idx = rrt_lib.find_closet_node(random_pt, tree)
        
        new_pt = random_pt-tree[min_parent_idx].pt
        new_pt = tree[min_parent_idx].pt + (new_pt / new_pt.norm()) * step_size
        
        min_cost  = rrt_lib.cost_np(tree[min_parent_idx], new_pt)
        new_node  = structs.Node(new_pt.x, new_pt.y, new_pt.z, -1, min_cost, min_parent_idx)
        
        if rrt_lib.is_collided(new_node.pt, tree[min_parent_idx].pt, world) == False:
            new_node = structs.Node(new_pt.x, new_pt.y, new_pt.z, -1, min_cost, min_parent_idx)
            tree.append(new_node)
            break

    # check to see if new node connects directly to end_node
    if rrt_lib.is_reached(new_node, end_node, step_size, world) == True:
        tree[-1].chi = 0  # mark node as connecting to end.
        return tree, True
   
    return tree, False