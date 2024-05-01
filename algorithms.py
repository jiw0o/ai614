from core import *
from treelib import Tree, Node
import random, math
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely import distance
import time
from core import Env
DRAWPAUSE = 0.0000001


class algo :
    def __init__(self, env:Env, max_iter:int = 1500, threshold:int = 10, headless:bool = False):
        self.env = env
        self.max_iter = max_iter
        self.threshold = threshold
        self.tree = PlanTree(env = self.env)
        self.plots = {}
        self.headless = headless
        
    def collision_check(self, p1:Point, p2:Point) -> bool:
        if self.env.obstacle is not None and len(self.env.obstacle.obstacles) > 0:
            line = LineString([(p1.x, p1.y), (p2.x, p2.y)])
            for obs in self.env.obstacle.obstacles :
                if obs.intersects(line) :
                    return True
        return False
    
    def reach(self, xnew: Node) -> bool:
        if xnew is not None:
            if distance(self.env.goal, xnew.data['pos']) < self.threshold :
                return True
        return False
    
    def plot_tree(self, parent_node:Node, child_node:Node, marker='.', color='lightblue') :
        if self.headless == True : return
        key = f"{child_node.identifier}"
        if key in self.plots :
            line = self.plots[key]['line']
            line[0].remove()
        else :
            self.plots[key] = {'line':None, 'mark':None}
        p1, p2 = parent_node.data['pos'], child_node.data['pos']
        self.plots[key]['line'] = self.env.ax.plot([p1.x, p2.x], [p1.y, p2.y], color=color)
        self.plots[key]['mark'] = self.env.ax.plot(p2.x, p2.y, marker=marker, color=color)
        plt.draw()
        plt.pause(DRAWPAUSE)
        
    def visualize(self, iter:int, time_consume, dst:Node = None):
        dist = 0
        tmp = self.headless
        self.headless = False
        if dst is not None :
            point = dst.data['pos']
            if dst is not None :
                while not dst.is_root() :
                    parent = self.tree.parent(dst.identifier)
                    self.plot_tree(parent, dst, marker="o", color='blue')
                    dist += distance(parent.data['pos'], dst.data['pos'])
                    dst = parent
            print(f"[Iteration] {iter}")
            print(f"[Destination] {point}")
            print(f"[Distance] {dist}")
            print(f"[Time Consumption] {time_consume}")
        else :
            print("Fail to find the path")
            print(f"[Iteration] {iter}")
            print(f"[Time Consumption] {time_consume}")
        self.env.marker()
        self.headless = tmp

     
class AlternativeSampling(algo) :
    def sampling(self, xtree: Point):
        xrand = Point(
            random.randint(0, self.env.width),
            random.randint(0, self.env.height)
        )
        while self.collision_check(xtree, xrand) :
            xrand = Point(
                random.randint(0, self.env.width),
                random.randint(0, self.env.height)
            )
        return xrand

    def extend(self, tree_node: Node, xrand: Point) -> Node:
        if not self.collision_check(tree_node.data['pos'], xrand):
            rand_node = self.tree.create_node(parent=tree_node, pos=xrand)
            self.plot_tree(tree_node, rand_node)
            return rand_node
        return None

    def planning(self) -> tuple[PlanTree, int, Node]:
        tmp = None
        for i in range(self.max_iter):
            tree_node = self.tree.random_sample()
            xrand = self.sampling(tree_node.data['pos'])
            rand_node = self.extend(tree_node, xrand)
            if rand_node is not None :
                tmp = self.temporal_draw(tmp, rand_node.data['pos'])
            if self.reach(rand_node) :
                return self.tree, i, rand_node
        return self.tree, i, None
    
    def temporal_draw(self, tmp, point) :
        if self.headless: return None
        if tmp is not None and len(tmp) > 0:
            for t in tmp : t.remove()
        tmp = self.env.ax.plot(point.x, point.y, color='pink', marker="*", markersize=10)
        plt.draw()
        plt.pause(DRAWPAUSE)
        return tmp


class DomainKnowledge(algo) :
    def sampling(self, xtree: Point):
        xrand = Point(
            random.randint(0, self.env.width),
            random.randint(0, self.env.height)
        )
        while self.collision_check(xtree, xrand) :
            xrand = Point(
                random.randint(0, self.env.width),
                random.randint(0, self.env.height)
            )
        return xrand

    def extend(self, tree_node: Node, xrand: Point) -> Node:
        if not self.collision_check(tree_node.data['pos'], xrand):
            rand_node = self.tree.create_node(parent=tree_node, pos=xrand)
            self.plot_tree(tree_node, rand_node)
            return rand_node
        return None

    def planning(self) -> tuple[PlanTree, int, Node]:
        tmp = None
        for i in range(self.max_iter):
            tree_node, _ = self.tree.goal_closest_node()
            xrand = self.sampling(tree_node.data['pos'])
            rand_node = self.extend(tree_node, xrand)
            if rand_node is not None :
                tmp = self.temporal_draw(tmp, rand_node.data['pos'])
            if self.reach(rand_node) :
                return self.tree, i, rand_node
        return self.tree, i, None
    
    def temporal_draw(self, tmp, point) :
        if self.headless: return None
        if tmp is not None and len(tmp) > 0:
            for t in tmp : t.remove()
        tmp = self.env.ax.plot(point.x, point.y, color='pink', marker="*", markersize=10)
        plt.draw()
        plt.pause(DRAWPAUSE)
        return tmp


class DomainKnowledge2(algo) :
    def sampling(self):
        x = random.randint(0, self.env.width)
        y = random.randint(0, self.env.height)
        return Point(x, y)

    def extend(self, xrand: Point) -> Node:
        xtree, _ = self.tree.closest_node(xrand)
        if not self.collision_check(xtree.data['pos'], xrand):
            rand_node = self.tree.create_node(parent=xtree, pos=xrand)
            self.plot_tree(xtree, rand_node)
            return rand_node
        return None
    
    def planning(self) -> tuple[PlanTree, int, Node]:
        tmp = None
        for i in range(self.max_iter):
            rand_node = self.extend(self.sampling())
            if rand_node is not None :
                tmp = self.temporal_draw(tmp, rand_node.data['pos'])
            if self.reach(rand_node) :
                return self.tree, i, rand_node
        return self.tree, i, None
    
    def temporal_draw(self, tmp, point) :
        if self.headless: return None
        if tmp is not None and len(tmp) > 0:
            for t in tmp : t.remove()
        tmp = self.env.ax.plot(point.x, point.y, color='pink', marker="*", markersize=10)
        plt.draw()
        plt.pause(DRAWPAUSE)
        return tmp


class RRT(algo) :
    step = 15
    
    def sampling(self):
        x = random.randint(0, self.env.width)
        y = random.randint(0, self.env.height)
        return Point(x, y)

    def extend(self, xrand: Point) -> Node:
        near_node, dist = self.tree.closest_node(xrand)
        xnear = near_node.data['pos']
        scale = self.step / dist
        dx = xrand.x - xnear.x 
        dy = xrand.y - xnear.y
        xnew = Point(xnear.x  + scale * dx, xnear.y + scale * dy)
        if not self.collision_check(xnear, xnew):
            new_node = self.tree.create_node(parent=near_node, pos=xnew)
            self.plot_tree(near_node, new_node)
            return new_node
        return None
    
    def planning(self) -> tuple[PlanTree, int, Node]:
        tmp = None
        for i in range(self.max_iter):
            xrand = self.sampling()
            new_node = self.extend(xrand)
            if new_node is not None :
                tmp = self.temporal_draw(tmp, xrand, new_node.data['pos'])
            if self.reach(new_node) :
                return self.tree, i, new_node
        return self.tree, i, None
    
    def temporal_draw(self, tmp, rand, new) :
        if self.headless: return None
        if tmp is not None and len(tmp) > 0:
            for t in tmp : t[0].remove()
        tmp = [
            self.env.ax.plot(new.x, new.y, color='pink', marker="*", markersize=10),
            self.env.ax.plot(rand.x, rand.y, color='purple', marker="o", markersize=10)
        ]
        plt.draw()
        plt.pause(DRAWPAUSE)
        return tmp


class RRTstar(algo) :
    step = 15
    gamma = 50
    min_r = 20
    
    def __init__(self, env: Env, max_iter: int = 1000, threshold: int = 10, headless=False):
        self.env = env
        self.max_iter = max_iter
        self.threshold = threshold
        self.tree = PlanTree(env = self.env, is_cost=True)
        self.plots = {}
        self.headless = headless
        self.rewire_time = 0
        self.choose_parent_time = 0
        
    def sampling(self):
        x = random.randint(0, self.env.width)
        y = random.randint(0, self.env.height)
        return Point(x, y)

    def near(self, xrand: Point):
        radius = max(self.gamma * (math.log(self.tree.num_nodes) / self.tree.num_nodes) ** 0.5, self.min_r)
        circle = plt.Circle((xrand.x, xrand.y), radius, color="pink", linestyle='--', fill=False, linewidth=1)
        return self.tree.near_nodes(xrand, radius), circle
    
    def steer(self, from_node: Node, to_point: Point) -> list[Point]:
        from_point = from_node.data['pos']
        dist = distance(from_point, to_point)
        if dist < self.step:
            return to_point
        else:
            scale = self.step / dist
            dx = to_point.x - from_point.x 
            dy = to_point.y - from_point.y
            return Point(from_point.x  + scale * dx, from_point.y + scale * dy)
    
    def new_cost(self, from_node: Node, to_point: Point):
        return from_node.data['cost'] + distance(from_node.data['pos'], to_point)

    def choose_parent(self, near_nodes: list[Node], xrand: Point):
        start = time.time()
        min_cost = float('inf')
        best_parent = None
        best_sigma = None
        for near_node in near_nodes:
            xnear = near_node.data['pos']
            sigma = self.steer(near_node, xrand)
            cost = self.new_cost(near_node, sigma)
            if cost < min_cost:
                min_cost = cost
                best_parent = near_node
                best_sigma = sigma
        self.choose_parent_time += (time.time() - start)
        return best_parent, best_sigma

    def rewire(self, near_nodes: list[Node], rand_node: Node):
        start = time.time()
        xrand = rand_node.data['pos']
        for near_node in near_nodes:
            xnear = near_node.data['pos']
            sigma = self.steer(rand_node, xnear)
            if self.new_cost(rand_node, sigma) < near_node.data['cost'] :
                if not self.collision_check(xnear, xrand):
                    self.tree.move_node(near_node.identifier, rand_node.identifier)
                    near_node.data['cost'] = self.new_cost(rand_node, xnear)
                    self.plot_tree(rand_node, near_node, color="springgreen")
        self.rewire_time += (time.time() - start)
    
    def planning(self):
        dst = None
        tmp = None
        c_tmp = None
        for i in range(self.max_iter):
            xrand = self.sampling()
            nearest_node, _ = self.tree.closest_node(xrand)
            xnew = self.steer(nearest_node, xrand)
            near_nodes, c = self.near(xnew)
            if xnew is not None :
                tmp, c_tmp = self.temporal_draw(tmp, c_tmp, xrand, xnew, c)
            parent_node, xnew = self.choose_parent(near_nodes, xnew)
            if parent_node is not None and not self.collision_check(parent_node.data['pos'], xnew):
                new_node = self.tree.create_node(parent=parent_node.identifier, pos=xnew, cost=self.new_cost(parent_node, xnew))
                self.plot_tree(parent_node, new_node)
                self.rewire(near_nodes, new_node)
                if self.reach(new_node) :
                    if dst is None :
                        dst = new_node
                    elif new_node.data['cost'] < dst.data['cost'] :
                        dst = new_node
        return self.tree, i, dst
    
    def temporal_draw(self, tmp, c_tmp, rand, new, circle) :
        if self.headless: return None, None
        if tmp is not None and len(tmp) > 0:
            for t in tmp : t[0].remove()
        if c_tmp is not None : c_tmp.remove()
        tmp = [
            self.env.ax.plot(new.x, new.y, color='pink', marker="*", markersize=10),
            self.env.ax.plot(rand.x, rand.y, color='purple', marker="o", markersize=10)
        ]
        c_tmp = self.env.ax.add_patch(circle)
        plt.draw()
        plt.pause(DRAWPAUSE)
        return tmp, c_tmp



class ImproveRRTstar(algo) :
    step = 15
    gamma = 50
    min_r = 20
    
    def __init__(self, env: Env, max_iter: int = 1000, threshold: int = 10, headless=False):
        self.env = env
        self.max_iter = max_iter
        self.threshold = threshold
        self.tree = PlanTree(env = self.env, is_cost=True)
        self.plots = {}
        self.headless = headless
        self.rewire_time = 0
        self.choose_parent_time = 0
        
    def sampling(self):
        x = random.randint(0, self.env.width)
        y = random.randint(0, self.env.height)
        return Point(x, y)

    def near(self, xrand: Point):
        radius = max(self.gamma * (math.log(self.tree.num_nodes) / self.tree.num_nodes) ** 0.5, self.min_r)
        circle = plt.Circle((xrand.x, xrand.y), radius, color="pink", linestyle='--', fill=False, linewidth=1)
        return self.tree.near_nodes(xrand, radius), circle
    
    def steer(self, from_node: Node, to_point: Point) -> list[Point]:
        from_point = from_node.data['pos']
        dist = distance(from_point, to_point)
        if dist < self.step:
            return to_point
        else:
            scale = self.step / dist
            dx = to_point.x - from_point.x 
            dy = to_point.y - from_point.y
            return Point(from_point.x  + scale * dx, from_point.y + scale * dy)
    
    def new_cost(self, from_node: Node, to_point: Point):
        return from_node.data['cost'] + distance(from_node.data['pos'], to_point)
    
    def global_cost(self, x: Point):
        return distance(x, self.env.goal)

    def choose_parent(self, near_nodes: list[Node], xrand: Point):
        start = time.time()
        min_cost = float('inf')
        best_parent = None
        for near_node in near_nodes:
            xnear = near_node.data['pos']
            cost = self.new_cost(near_node, xrand) + self.global_cost(xnear)
            if cost < min_cost:
                min_cost = cost
                best_parent = near_node
        self.choose_parent_time += (time.time() - start)
        return best_parent

    def rewire(self, near_nodes: list[Node], rand_node: Node):
        start = time.time()
        xrand = rand_node.data['pos']
        for near_node in near_nodes:
            xnear = near_node.data['pos']
            if self.new_cost(rand_node, xnear) < near_node.data['cost'] :
                if not self.collision_check(xnear, xrand):
                    self.tree.move_node(near_node.identifier, rand_node.identifier)
                    near_node.data['cost'] = self.new_cost(rand_node, xnear)
                    self.plot_tree(rand_node, near_node, color="springgreen")
        self.rewire_time += (time.time() - start)
    
    def planning(self):
        dst = None
        tmp = None
        c_tmp = None
        for i in range(self.max_iter):
            xrand = self.sampling()
            nearest_node, _ = self.tree.closest_node(xrand)
            xnew = self.steer(nearest_node, xrand)
            near_nodes, c = self.near(xnew)
            if xnew is not None :
                tmp, c_tmp = self.temporal_draw(tmp, c_tmp, xrand, xnew, c)
            parent_node = self.choose_parent(near_nodes, xnew)
            if parent_node is not None and not self.collision_check(parent_node.data['pos'], xnew):
                new_node = self.tree.create_node(parent=parent_node.identifier, pos=xnew, cost=self.new_cost(parent_node, xnew))
                self.plot_tree(parent_node, new_node)
                self.rewire(near_nodes, new_node)
                if self.reach(new_node) :
                    if dst is None :
                        dst = new_node
                    elif new_node.data['cost'] < dst.data['cost'] :
                        dst = new_node
        return self.tree, i, dst
    
    def temporal_draw(self, tmp, c_tmp, rand, new, circle) :
        if self.headless: return None, None
        if tmp is not None and len(tmp) > 0:
            for t in tmp : t[0].remove()
        if c_tmp is not None : c_tmp.remove()
        tmp = [
            self.env.ax.plot(new.x, new.y, color='pink', marker="*", markersize=10),
            self.env.ax.plot(rand.x, rand.y, color='purple', marker="o", markersize=10)
        ]
        c_tmp = self.env.ax.add_patch(circle)
        plt.draw()
        plt.pause(DRAWPAUSE)
        return tmp, c_tmp

