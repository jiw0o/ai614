from core import *
from algorithms import *
import time

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
a4.first_solution_stop = False
a5 = ImproveRRTstar(env)
a5.first_solution_stop = False

def job(a:algo) :
    a.max_iter = 1000
    a.headless = True
    tree, iter, dst, t = a.planning()
    a.visualize(iter=iter, time_consume=t, dst=dst)
    print(f"[Number of node] {tree.num_nodes}")
    plt.show()
    
job(a1)