#super rough and temporary testing file
import time
import networkx as nx
import detector_functions as df
import random as rand
from itertools import product

size = 50
g = nx.Graph()
points = [(rand.randint(0,size),rand.randint(0,size)) for _ in range(2000)]


t0 = time.clock()
edges = df.get_edges(points)
t1 = time.clock()
g.add_edges_from(edges)
groups = nx.connected_components(g)
t2 = time.clock()

print t1-t0
print t2-t1
