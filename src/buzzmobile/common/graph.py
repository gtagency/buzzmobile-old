# Copyright (c) The Agency at Georgia Tech College of Computing 2013
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# This file contains code for constructing and searching a graph.  Vertices
# in the graph will hold information about that vertex.  This information
# will be defined by the code that constructs the graph.

import util
from collections import deque as Queue
import heapq

class Edge:
    def __init__(self, u, v, w=1):
        self.u = u
        self.v = v
        self.weight = w
    
    # Return a reversed version of this edge
    # keeping the weight the same.  This is
    # commonly used for detecting strongly
    # connected components, and for using common
    # shortest path algorithms to find single dest
    # many source paths
    def reversed(self):
        return Edge(self.v, self.u, self.weight)


class Vertex:
    def __init__(self, i):
        self.info = i

    def __eq__(self, other):
        return self.info == other.info

    def __repr__(self): 
        return str("Vertex: %s" % self.info)


class Graph:
    def __init__(self, v, e):
        self.vertices = v
        self.edges = e

    # Returns list of tuples (v, w) for vertex u
    def getSuccessors(self, u):
        # NOTE: this is dumb and can be optimized        
        return [e for e in self.edges if e.u == u]

    def search(self, struct, start, proc=lambda e: None):
        print struct.hasMore()
        
        struct.push(Edge(start, start, 0))
        while (struct.hasMore()):
            edge = struct.pop()
            print "Processing edge (%s, %s) with edge weight %d" % (edge.u, edge.v, edge.weight)
            proc(edge)
            successors = self.getSuccessors(edge.v)
#            print "Checking ", node, " with cost ", node.cost, ", ", struct.count(), " left"
            for successor in successors:
                struct.push(successor)

class SearchStruct:
    
    def hasMore(self):
        util.raiseNotDefined()

    def push(self, edge):
        util.raiseNotDefined()
        
    def pop(self):
        util.raiseNotDefined()

"""
  An implementation of a simple stack (LIFO), used to implement DFS with general graph search.
  This implementation maintains a visited list, to ensure the graph search is efficient.

  This will enable the search to run in O(|V| + |E|) time.
"""
class SearchStack(SearchStruct):

    def __init__(self):
        self.stack = []
        self.visited = []
    
    def hasMore(self):
        return self.stack

    def push(self, edge):
        node = edge.v
        if (node not in self.visited):
          self.stack.append(edge)  
          "Add to the visited list here, because we use it above for deduplication"
          self.visited.append(node)

    def pop(self):
        edge = self.stack.pop()
        return edge

    def count(self):
        return len(self.stack)
        
"""
  An implementation of a simple queue (FIFO), used to implement BFS with general graph search.
  This implementation maintains a visited list, to ensure the graph search is efficient.
  
  This will enable the search to run in O(|V| + |E|) time.
"""
class SearchQueue(SearchStruct):

    def __init__(self):
        self.queue = Queue()
        self.visited = [];

    def hasMore(self):
        return self.queue

    def push(self, edge):
        node = edge.v
        if (node not in self.visited):
          self.queue.appendleft(edge)
          "Add to the visited list here, because we use it above for deduplication"
          self.visited.append(node)
        else:
          print "Ignoring node ", node

    def pop(self):
        edge = self.queue.pop()
        return edge
    
    def count(self):
        return len(self.queue)

#Min Heap based
class SearchPriorityQueue(SearchStruct):
    
    def __init__(self):
        self.heap = []
        self.visited = [];

    def hasMore(self):
        return self.heap

    def push(self, edge):
        node = edge.v
        # NOTE: this does not work if two edges with the same source and destination are pushed
        # in the same getSuccessor step with different edge weights.  If we can avoid
        # that situation, we're golden.
        if (node not in self.visited):
          heapq.heappush(self.heap, (edge.weight, edge))
          "Add to the visited list here, because we use it above for deduplication"
          self.visited.append(node)
        else:
          print "Ignoring node ", node

    def pop(self):
        edge = heapq.heappop(self.heap)[1]
        return edge
    
    def count(self):
        return len(self.heap)
    
"""
    This is an implementation of Dijkstra's Shortest path algorithm.
    
"""

def shortestPath(graph, s):
    dist = {}
    prev = {}
    
    def proc(e):
        if (dist[e.v] > dist[e.u] + e.weight):
            dist[e.v] = dist[e.u] + e.weight
            prev[e.v] = e.u

    graph.search(SearchPriorityQueue(), s, proc)
