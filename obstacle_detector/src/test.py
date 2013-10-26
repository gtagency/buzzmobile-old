#super hacky and temporary testing file
import time
import networkx as nx
import detector_functions as df
import random as rand
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
from itertools import product

size = 7
g = nx.Graph()
points = [(rand.random()*size,rand.random()*size) for _ in range(50)]

t0 = time.clock()
edges = df.get_edges(points)
t1 = time.clock()
g.add_edges_from(edges)
groups = nx.connected_components(g)
t2 = time.clock()

print t1-t0
print t2-t1
print "Num groups {}".format(len(groups))

fig, ax = plt.subplots()

for group in groups:
    x = [p[0] for p in group]
    y = [p[1] for p in group]
    hull = df.graham_scan(group)
    plt.plot(x,y,marker='o',ls='-')
    polygon = Polygon(hull, True, alpha=.1)
    ax.add_patch(polygon)

plt.xlim([-size/10.0,size*1.1])
plt.ylim([-size/10.0,size*1.1])
plt.show()
