from treelib import Tree, Node
import random
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString
from shapely import distance

class Obstacle:
    obstacles = [
        Polygon([(10, 60), (40, 120), (120, 20)]),
        Polygon([(30, 170), (90, 230), (100, 170)]),
        Polygon([(140, 70), (90, 150), (220, 110), (220, 10)]),
        Polygon([(180, 180), (210, 180), (210, 210), (180, 210)]),
    ]
    

class Env:
    def __init__(
        self,
        width:int = 250,
        height:int = 250,
        init:Point = Point(30, 30),
        goal:Point = Point(220, 220),
        obstacle: Obstacle = None
        ):
        self.width = width
        self.height = height
        self.init = init
        self.goal = goal
        self.obstacle = obstacle
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.marker()
        plt.draw()
        plt.pause(0.0000001)

    def marker(self) :
        self.ax.axis([0, self.width, 0, self.height])
        self.ax.plot(self.init.x, self.init.y, marker="v", color="red", markersize=10)
        self.ax.plot(self.goal.x, self.goal.y, marker="X", color="olivedrab", markersize=10)
        c = plt.Circle((self.goal.x, self.goal.y), 10, color="olivedrab", linestyle='--', fill=False, linewidth=1)
        self.ax.add_patch(c)
        if self.obstacle is not None and len(self.obstacle.obstacles) > 0:
            for obs in self.obstacle.obstacles :
                xe, ye = obs.exterior.xy
                self.ax.fill(xe, ye, color='tan')
        plt.draw()
        plt.pause(0.0000001)
                

class PlanTree(Tree):
    def __init__(
        self,
        env: Env,
        is_cost=False,
        tree=None, deep=False, node_class=None, identifier=None
        ):
        super().__init__(tree, deep, node_class, identifier)
        self.num_nodes = 1
        self.env = env
        self.is_cost = is_cost
        if is_cost :
            super().create_node(tag="Start", identifier=1, data={'pos': env.init, 'cost': 0})
        else :
            super().create_node(tag="Start", identifier=1, data={'pos': env.init})
                
    def print_tree(self) :
        print(self.show(stdout=False))

    def create_node(self, parent:int = None, pos:Point = None, cost:float = 0):
        self.num_nodes += 1
        if self.is_cost :
            return super().create_node(
                tag=self.num_nodes,
                identifier=self.num_nodes,
                parent=parent,
                data={"pos": pos, "cost": cost}
                )
        return super().create_node(
            tag=self.num_nodes,
            identifier=self.num_nodes,
            parent=parent,
            data={"pos": pos}
            )
        
    def random_sample(self):
        return random.sample(self.all_nodes(), k=1)[0]

    def goal_closest_node(self):
        goal = self.env.goal
        nodes = self.all_nodes()
        c_node = self.get_node(1)
        c_dist = distance(c_node.data['pos'], goal)
        for node in nodes :
            dist = distance(node.data['pos'], goal)
            if dist < c_dist :
                c_node = node
                c_dist = dist
        return c_node, c_dist

    def closest_node(self, x: Point):
        nodes = self.all_nodes()
        c_node = self.get_node(1)
        c_dist = distance(c_node.data['pos'], x)
        for node in nodes :
            dist = distance(node.data['pos'], x)
            if dist < c_dist :
                c_node = node
                c_dist = dist
        return c_node, c_dist

    def near_nodes(self, x: Point, radius:float) -> list[Node]:
        nodes = self.all_nodes()
        c_nodes = []
        for node in nodes :
            if distance(node.data['pos'], x) < radius :
                c_nodes.append(node)
        return c_nodes
