import random
import rrt_lazy
import rrt_connect
import rrt_extend
import rrt_star
import structs

def initialise_world(config):
    world_size = config['world_size']
    if config['is_random_world'] == True:
        return create_random_world(config["obstalces_num"], world_size, world_size, world_size)
    
    return create_world(world_size, world_size, world_size)

def create_random_world(obstacles_num, length, width, height):
    world = structs.World(length, width, height)
    max_radius = 5 * min([length, width, height]) / obstacles_num
    
    for i in range(obstacles_num):

        radius = max_radius * random.random()
        cx = radius + (width - 2 * radius) * random.random()
        cy = radius + (length - 2 * radius) * random.random()
        cz = radius + (height - 2 * radius) * random.random()

        world.obstacles.append(structs.Obstacle(cx, cy, cz, radius))

    return world

def create_world(length, width, height): 
    world = structs.World(length, width, height)

    min_edge = min([length, width, height])
    max_edge = min([length, width, height])

    maxRadius = min(min_edge / 8, max_edge / 10)

    ax = width / 4
    ay = length / 4
    az = height / 4

    dx = [2, 3, 1, 1, 1, 3, 3, 3, 1]
    dy = [2, 3, 1, 1, 3, 1, 3, 1 ,3]
    dz = [2, 3, 1, 3, 1, 1, 1, 3 ,3]

    for i in range(9):
        cx = ax * dx[i]
        cy = ay * dy[i]
        cz = az * dz[i]

        world.obstacles.append(structs.Obstacle(cx, cy, cz, maxRadius))
    
    return world

def find_path_by_rrt(uav, world, config):
    start_node = structs.Node(uav.start_pt.x, uav.start_pt.y, uav.start_pt.z)
    end_node = structs.Node(uav.end_pt.x, uav.end_pt.y, uav.end_pt.z)

    if config['rrt_type'] == 'lazy':
        return rrt_lazy.run_lazy_rrt(config, world, start_node, end_node)

    if config['rrt_type'] == 'connect':
        return rrt_connect.run_rrt_connect(config, world, start_node, end_node)

    if config['rrt_type'] == 'extend':
        return rrt_extend.run_rrt_extend(config, world, start_node, end_node)

    if config['rrt_type'] == 'star':
        return rrt_star.run_rrt_star(config, world, start_node, end_node)

    print("Invalid RRT type!")

def generate_random_node(world: structs.World): 
    node = structs.Node(world.width * random.random(), world.length * random.random(), world.height * random.random())
    
    # check collision with obstacle
    while is_collided(node, node, world):
        node = structs.Node(world.width * random.random(), world.length * random.random(), world.height * random.random())

    return node

def is_collided(pt1: structs.Point, pt2: structs.Point, world: structs.World):
    if pt1.x > world.width or pt1.y > world.length or pt1.z > world.height:
        return True

    for sigma in [0, 0.2, 0.4, 0.6, 0.8, 1.0]:
        p = [sigma * pt1.x + (1 - sigma) * pt2.x, sigma * pt1.y + (1 - sigma) * pt2.y, sigma * pt1.z + (1 - sigma) * pt2.z]
        
        for obs in world.obstacles:
            dist = (p[0] - obs.x) ** 2 + (p[1] - obs.y) ** 2 + (p[2] - obs.z) ** 2
            
            if dist <= obs.radius * obs.radius:
                return True
    
    return False

def find_closet_node(pt: structs.Point, tree):
    idx = 0
    dist = pt.dist_to(tree[0])

    for i in range(1, len(tree)):
        cur = pt.dist_to(tree[i])
        if cur < dist:
            dist = cur
            idx = i

    return idx

def cost_np(from_node: structs.Node, to_point: structs.Point):
    return from_node.cost + (from_node - to_point).norm()

def line_cost(from_node, to_point):
    return (from_node - to_point).norm()

def is_reached(node, end_node, step_size, world):
    return (node - end_node).norm() < step_size and is_collided(node.pt, end_node.pt, world) == False

def find_min_path(tree, end_node):
    # Find nodes that connect to the end_node
    connecting_nodes = []
    for node in tree:
        if node.chi == 0:  # Check if the node connects to the end_node
            connecting_nodes.append(node)

    # Find the minimum cost node among connecting nodes
    if not connecting_nodes:
        return []
    
    idx = 0
    cost = connecting_nodes[0].cost
    for i in range(1, len(connecting_nodes)):
        if connecting_nodes[i].cost < cost:
            idx = i
            cost = connecting_nodes[i].cost

    # Initialize the path with the minimum cost node and the end node
    path = [connecting_nodes[idx], end_node]

    # Trace back to construct the path using parent nodes
    parent_id = connecting_nodes[idx].parent
    while parent_id > 0:  # Continue until reaching the root node
        parent_id = tree[parent_id].parent
        path.insert(0, tree[parent_id])  # Prepend the parent node to the path

    return path