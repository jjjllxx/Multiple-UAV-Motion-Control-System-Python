class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __iter__(self):
        return [self.x, self.y, self.z]
    
    def norm(self):
        return (self.x * self.x + self.y * self.y + self.z * self.z) ** 0.5
    
    def __add__(self, other):
        if isinstance(other, Point):
            return Point(self.x + other.x, self.y + other.y, self.y + other.y)
        if isinstance(other, Node):
            return self.__sub__(other.pt)
        raise("Type not supported")
    
    def __sub__(self, other):
        if isinstance(other, Point):
            return Point(self.x - other.x, self.y - other.y, self.y - other.y)
        if isinstance(other, Node):
            return self.__sub__(other.pt)
        raise("Type not supported")
    
    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Point(self.x * other, self.y * other, self.z * other)
        raise("Type not supported")
    
    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Point(self.x / other, self.y / other, self.z / other)
        raise("Type not supported")

    def dist_to(self, other):
        if isinstance(other, Point):
            return (self - other).norm()
        if isinstance(other, Node):
            return (self - other.pt).norm()
        raise("Type not supported")
    
    def dot(self, other):
        if isinstance(other, Point):
            return self.x * other.x + self.y + other.y + self.z * other.z
        if isinstance(other, Node):
            return self.x * other.pt.x + self.y + other.pt.y + self.z * other.pt.z
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

    def __sub__(self, other):
        if isinstance(other, Point):
            return Point(self.pt.x - other.x, self.pt.y - other.y, self.pt.z - other.z)
        if isinstance(other, Node):
            return self.__sub__(other.pt)
        raise("Type not supported")

class World:
    def __init__(self, width, length,  height):
        self.width = width
        self.length = length
        self.height = height
        self.obstacles = []

class UAV:
    def __init__(self, start_pt: Point, end_pt: Point):
        self.start_pt = start_pt
        self.end_pt = end_pt