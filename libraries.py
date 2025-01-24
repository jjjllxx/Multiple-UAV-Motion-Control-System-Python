import random
import matplotlib.pyplot as plt
import numpy as np

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __iter__(self):
        return [self.x, self.y, self.z]
    
    def norm(self):
        return (self.x * self.x + self.y * self.y + self.z * self.z) ** 0.5
    
    def __add__(self, dest):
        if isinstance(dest, Point):
            return Point(self.x + dest.x, self.y + dest.y, self.y + dest.y)
        if isinstance(dest, Node):
            return self.__sub__(dest.pt)
        raise("Type not supported")
    
    def __sub__(self, dest):
        if isinstance(dest, Point):
            return Point(self.x - dest.x, self.y - dest.y, self.y - dest.y)
        if isinstance(dest, Node):
            return self.__sub__(dest.pt)
        raise("Type not supported")
    
    def __mul__(self, dest):
        if isinstance(dest, (int, float)):
            return Point(self.x * dest, self.y * dest, self.z * dest)
        raise("Type not supported")
    
    def __truediv__(self, dest):
        if isinstance(dest, (int, float)):
            return Point(self.x / dest, self.y / dest, self.z / dest)
        raise("Type not supported")

    def dist_to(self, dest):
        if isinstance(dest, Point):
            return (self - dest).norm()
        if isinstance(dest, Node):
            return (self - dest.pt).norm()
        raise("Type not supported")
    
class Obstacle:
    def __init__(self, x, y, z, radius):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius

class Node:
    def __init__(self, x, y, z, chi=-1, cost=0, parent=-1):
        self.pt = Point(x, y, z)

        self.chi = chi
        self.cost = cost
        self.parent = parent

    def __sub__(self, dest):
        if isinstance(dest, Point):
            return Point(self.pt.x - dest.x, self.pt.y - dest.y, self.pt.z - dest.z)
        if isinstance(dest, Node):
            return self.__sub__(dest.pt)
        raise("Type not supported")

class World:
    def __init__(self, width, length,  height):
        self.width = width
        self.length = length
        self.height = height
        self.obstacles = []

def create_random_world(NumObstacles, length, width, height):
    world = World(length, width, height)
    max_radius = 5 * min([length, width, height]) / NumObstacles
    
    for i in range(NumObstacles):

        radius = max_radius * random.random()
        cx = radius + (width - 2 * radius) * random.random()
        cy = radius + (length - 2 * radius) * random.random()
        cz = radius + (height - 2 * radius) * random.random()

        world.obstacles.append(Obstacle(cx, cy, cz, radius))

    return world

def create_world(length, width, height): 
    world = World(length, width, height)

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

        world.obstacles.append(Obstacle(cx, cy, cz, maxRadius))
    
    return world

def generate_random_node(world: World): 
    node = Node(world.width * random.random(), world.length * random.random(), world.height * random.random())
    
    # check collision with obstacle
    while is_collided(node, node, world):
        node = Node(world.width * random.random(), world.length * random.random(), world.height * random.random())

    return node

def is_collided(node: Node, parent: Node, world: World):
    if node.pt.x > world.width or node.pt.y > world.length or node.pt.z > world.height:
        return True

    for sigma in [0, 0.2, 0.4, 0.6, 0.8, 1.0]:
        # Calculate the point p based on sigma
        p = [sigma * node.pt.x + (1 - sigma) * parent.pt.x, sigma * node.pt.y + (1 - sigma) * parent.pt.y, sigma * node.pt.z + (1 - sigma) * parent.pt.z]
        
        # Check for collisions with each obstacle
        for obs in world.obstacles:
            # Calculate the distance between p and the obstacle center
            dist = (p[0] - obs.x) ** 2 + (p[1] - obs.y) ** 2 + (p[2] - obs.z) ** 2
            
            if dist <= obs.radius * obs.radius:
                return True
    
    return False

def find_closet_node(pt: Point, tree):
    idx = 0
    dist = pt.dist_to(tree[0])

    for i in range(1, len(tree)):
        cur = pt.dist_to(tree[i])
        if cur < dist:
            dist = cur
            idx = i

    return idx

def cost_np(from_node: Node, to_point: Point):
    return from_node.cost + (from_node - to_point).norm()

def line_cost(from_node, to_point):
    return (from_node - to_point).norm()

def find_min_path(tree, end_node):
    # Find nodes that connect to the end_node
    connecting_nodes = []
    for node in tree:
        if node.chi == 0:  # Check if the node connects to the end_node
            connecting_nodes.append(node)

    # Find the minimum cost node among connecting nodes
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

def plot_result(world: World, tree, path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim(0, world.width)
    ax.set_ylim(0, world.length)
    ax.set_zlim(0, world.height)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('RRT Algorithm (3D)')

    # Plot obstacles
    for obs in world.obstacles:
        u, v = np.mgrid[0:2 * np.pi:10j, 0:np.pi:10j]
        X = obs.radius * np.sin(v) * np.cos(u)
        Y = obs.radius * np.sin(v) * np.sin(u)
        Z = obs.radius * np.cos(v)
        ax.plot_surface(X + obs.x, Y + obs.y, Z + obs.z, color=[0.5, 0.2, 0.3])

    idx = len(tree) - 1  # Start from the last node index
    for idx in range(len(tree) - 1, -1, -1):
        branch = []
        node = tree[idx]  # Current node
        branch.append(node)  # Add current node to the branch
        
        parent_id = node.parent
        while parent_id > 0:  # Iterate until reaching the root node
            branch.append(tree[parent_id])  # Add parent node
            parent_id = tree[parent_id].parent  # Get the parent of the current node

        # Extract X, Y, Z coordinates
        X = [node.pt.x for node in branch]
        Y = [node.pt.y for node in branch]
        Z = [node.pt.z for node in branch]

        ax.plot(X, Y, Z, color='red', linewidth=0.5, marker='.', markeredgecolor='green')

    # Plot the path
    ax.plot([node.pt.x for node in path], [node.pt.y for node in path], [node.pt.z for node in path], color='black', linewidth=3)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Expanded Tree (3D)')
    plt.grid(True)
    plt.show()