import rrt_lib
import random
import math

def run_rrt_star(seg_length, is_random_world, samples):
    world = rrt_lib.create_random_world(100, 100, 100, 100) if is_random_world == True else rrt_lib.create_world(100, 100, 100)

    start_node = rrt_lib.Node(5, 5, 5)
    end_node = rrt_lib.Node(95, 95, 95)

    tree = [start_node]
    trees = []
    div = 8
    radius = 10

    if rrt_lib.is_reached(start_node, end_node, seg_length, world) == True:
        path = [start_node, end_node]
    else:
        if samples > 0:
            draw = math.floor(samples / div)
            for i in range(samples):
                tree, is_found = extend_tree(tree, end_node, seg_length, radius, world)
                for j in range(div):
                    if i == draw * (j + 1):
                        trees.append(tree)

                if i + 1 == samples:
                    trees.append(tree)

        else: 
            is_found = False
            while not is_found:
                tree, is_found = extend_tree(tree, end_node, seg_length, radius, world)

    for i in range(len(trees)):
        path = rrt_lib.find_min_path(trees[i], end_node)
        rrt_lib.plot_result(world, trees[i], path) if path else print(f"COULD NOT FIND A CONNECTING TREE TILL {i + 1}/{div}th SAMPLES SO NOT DRAWING PATH")
        


def extend_tree(tree, end_node, seg_length, radius, world):
    while True:
        random_pt = rrt_lib.Point(world.width * random.random(), world.length * random.random(), world.height * random.random()) 
        
        min_parent_idx = rrt_lib.find_closet_node(random_pt, tree)
        
        new_pt = random_pt-tree[min_parent_idx].pt
        new_pt = tree[min_parent_idx].pt + (new_pt / new_pt.norm()) * seg_length
        
        min_cost  = rrt_lib.cost_np(tree[min_parent_idx], new_pt)
        new_node  = rrt_lib.Node(new_pt.x, new_pt.y, new_pt.z, -1, min_cost, min_parent_idx)
        
        if rrt_lib.is_collided(new_node.pt, tree[min_parent_idx].pt, world) == False:

            near_idx = []
            for i in range(len(tree)):
                if (tree[i] - new_pt).norm() <= radius:
                    near_idx.append(i)
            
            size_near = len(near_idx)
            if size_near > 1:
                for idx in near_idx:
                    if rrt_lib.is_collided(new_node.pt, tree[idx].pt, world) == False:
                        
                        cost_near = tree[idx].cost + rrt_lib.line_cost(tree[idx], new_pt)
                        
                        if  cost_near < min_cost:
                            min_cost = cost_near
                            min_parent_idx = idx

            
            new_node = rrt_lib.Node(new_pt.x, new_pt.y, new_pt.z, -1, min_cost, min_parent_idx)
            tree.append(new_node)
            new_node_idx = len(tree) - 1
            
            if size_near > 1:
                for idx in near_idx:
                    if tree[idx].cost > min_cost + rrt_lib.line_cost(tree[idx], new_pt) and rrt_lib.is_collided(tree[idx].pt, new_node.pt, world) == True:
                        tree[idx].parent = new_node_idx

            break
    
    # check to see if new node connects directly to end_node
    if rrt_lib.is_reached(new_node, end_node, seg_length, world) == True:
        tree[-1].chi = 0  # mark node as connecting to end.
        return tree, True
   
    return tree, False