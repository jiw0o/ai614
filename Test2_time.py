from core import *
from algorithms import *
import time
from copy import deepcopy
from shapely import distance

obs = Obstacle()
obs.obstacles = [
        Polygon([(80, 80), (170, 80), (170, 170), (170, 170), (80, 170), (80, 150), (150, 150), (150, 100), (80, 100)]),
    ]
env = Env(obstacle=obs, init=Point(125, 125), goal=Point(210, 125))

start = time.time()
a1 = AlternativeSampling(env)
a2 = DomainKnowledge(env)
a3 = RRT(env)
a4 = RRTstar(env)
a4.first_solution_stop = True
a5 = ImproveRRTstar(env)
a5.first_solution_stop = True

def job(a:algo, iteration) :
    avg_time_consume = 0
    avg_node = 0
    avg_iter = 0
    avg_dist = 0
    fail = 0
    for _ in range(iteration) :
        tmp = deepcopy(a)
        tmp.max_iter = 3000
        tmp.headless = True
        tree, iter, dst, t = tmp.planning()
        if dst is None :
            fail += 1
        else :
            if dst is not None :
                while not dst.is_root() :
                    parent = tmp.tree.parent(dst.identifier)
                    avg_dist += distance(parent.data['pos'], dst.data['pos'])
                    dst = parent
            avg_time_consume += t
            avg_node += tree.num_nodes
            avg_iter += iter
    avg_time_consume /= (iteration - fail)
    avg_node /= (iteration - fail)
    avg_iter /= (iteration - fail)
    avg_dist /= (iteration - fail)
    return avg_time_consume, avg_node, avg_iter, avg_dist, fail

def job_star(a:algo, iteration) :
    avg_time_consume = 0
    avg_rewire = 0
    avg_parent = 0
    avg_node = 0
    avg_iter = 0
    avg_dist = 0
    fail = 0
    for _ in range(iteration) :
        tmp = deepcopy(a)
        tmp.max_iter = 3000
        tmp.headless = True
        tree, iter, dst, t = tmp.planning()
        if dst is None :
            fail += 1
        else :
            if dst is not None :
                while not dst.is_root() :
                    parent = tmp.tree.parent(dst.identifier)
                    avg_dist += distance(parent.data['pos'], dst.data['pos'])
                    dst = parent
            avg_time_consume += t
            avg_rewire += tmp.rewire_time
            avg_parent += tmp.choose_parent_time
            avg_node += tree.num_nodes
            avg_iter += iter
    avg_time_consume /= (iteration - fail)
    avg_rewire /= (iteration - fail)
    avg_parent /= (iteration - fail)
    avg_node /= (iteration - fail)
    avg_iter /= (iteration - fail)
    avg_dist /= (iteration - fail)
    return avg_time_consume, avg_rewire, avg_parent, avg_node, avg_iter, avg_dist, fail

#'''
t, n, i, d, f = job(a1, 50)
print(f"Average Distance to Goal : {d: .2f}")
print(f"Average Time to Find The Path : {t: .4f}s")
print(f"Average Number of Node : {n: .2f}")
print(f"Average Number of Iteration : {i: .2f} times")
print(f"Number of fails: {f}/50")
'''
t, r, p, n, i, d, f = job_star(a5, 50)
print(f"Average Distance to Goal : {d: .2f}")
print(f"Average Time to Find The Path : {t: .4f}s")
print(f"Average Rate of Rewire : {r/t*100: .2f}%")
print(f"Average Rate of ChooseParent : {p/t*100: .2f}%")
print(f"Average Number of Node : {n: .2f}")
print(f"Average Number of Iteration : {i: .2f} times")
print(f"Number of fails: {f}/50")
'''
#'''