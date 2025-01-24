import libraries
import random

def run_lazy_rrt(seg_length, is_random_world):
    world = libraries.create_random_world(100, 100, 100, 100) if is_random_world == True else libraries.create_world(100, 100, 100)

    start_node = libraries.Node(5, 5, 5)
    end_node = libraries.Node(95, 95, 95)

    tree = [start_node]

    # check to see if start_node connects directly to end_node
    if (start_node - end_node).norm() < seg_length and libraries.is_collided(start_node, end_node, world) == False:
        path = [start_node, end_node]
    else:
        tree = extend_lazy_tree(tree, end_node, seg_length, world)
  
    # find path with minimum cost to end_node
    lazy_path = libraries.find_min_path(tree, end_node)
    path = repair_lazy_path(lazy_path, seg_length, world)

    libraries.plot_result(world, tree, path)

def repair_lazy_path(lazy_path, seg_length, world):
    path = []

    is_start = False
    is_end = False

    for i in range(len(lazy_path) - 1):
        has_collision = libraries.is_collided(lazy_path[i+1], lazy_path[i], world)
        
        if  has_collision == True and is_start == False:
            start_collision_node = lazy_path[i]
            is_start = True
            is_end = True

        elif is_end == True and has_collision == False:
            is_start = False
            is_end = False
            tree = [start_collision_node]
            end_node = lazy_path[i]
            tree = extend_tree(tree, end_node, seg_length, world)

            path += libraries.find_min_path(tree, end_node)
            
        elif is_start == False and is_end == False and has_collision == False:
            path.append(lazy_path[i])
            path.append(lazy_path[i + 1])

    return path

def extend_tree(tree, end_node, seg_length, world: libraries.World):
    while True:
        while True:
            random_pt = libraries.Point(world.width * random.random(), world.length * random.random(), world.height * random.random()) 

            idx = libraries.find_closet_node(random_pt, tree)
            
            new_point = random_pt - tree[idx]
            new_point = tree[idx].pt + (new_point / new_point.norm()) * seg_length
            
            min_cost  = libraries.cost_np(tree[idx], new_point)
            new_node  = libraries.Node(new_point.x, new_point.y, new_point.z, -1, min_cost, idx)
            
            if libraries.is_collided(new_node, tree[idx], world) == False:
                tree.append(new_node)
                break
        
        if (new_node - end_node).norm() < seg_length and libraries.is_collided(new_node, end_node, world) == False:
            tree[-1].chi = 0
            return tree
    
def extend_lazy_tree(tree, end_node, seg_length, world: libraries.World):
    while True:
        random_pt = libraries.Point(world.width * random.random(), world.length * random.random(), world.height * random.random())

        idx = libraries.find_closet_node(random_pt, tree)
        
        new_point = random_pt - tree[idx]
        new_point = tree[idx].pt + (new_point / new_point.norm()) * seg_length
        
        min_cost  = libraries.cost_np(tree[idx], new_point)
        new_node  = libraries.Node(new_point.x, new_point.y, new_point.z, -1, min_cost, idx)

        tree.append(new_node)

        if (new_node - end_node).norm() < seg_length and libraries.is_collided(new_node, end_node, world) == False:
            tree[-1].chi = 0

            return tree