
import heapq
from distance import *

class SearchNode(object):

    def __init__(self, state, cost=0, fVal = 0, cameFrom=None):
        self.state = state
        self.cost = 0
        self.fVal = fVal
        self.cameFrom = cameFrom

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return self.fVal < other.fVal

class AStarSearch(object):

    def __init__(self, network):
        self.network = network
 
    def path(self, source, dest):
        frontier = []
        visited  = []
        found    = None
        heapq.heappush(frontier, SearchNode(source))
        print frontier
        while frontier:
            node = heapq.heappop(frontier)
            state = node.state
            #print "Searching from: ", state
            if state == dest:
                #print "Found destination"
                found = node
                break
            visited.append(node)
            outgoing = self.network.getOutboundEdges(state)
            #print "Outgoing edges: ", outgoing
            for _, c in outgoing:
                #print "Checking child", c 
                g = node.cost + euclidean(state.latitude, state.longitude, c.latitude, c.longitude)
                f = g + euclidean(c.latitude, c.longitude, dest.latitude, dest.longitude)
                newNode = SearchNode(c, g, f, node)
                
                if (newNode not in visited  or f < visited[visited.index(newNode)].fVal) and \
                   (newNode not in frontier or f < frontier[frontier.index(newNode)].fVal):
                    #print "Adding node with cost", f
                    if newNode in visited:
                        visited.remove(newNode)
                    if newNode in frontier:
                        frontier.remove(newNode)
                    heapq.heappush(frontier, newNode)
        path = []
        while found:
            path.append(found.state)
            found = found.cameFrom
        path.reverse()
        #if path:
        #    print "Path found: ", path
        #else:
        #    print "No path found!"
        return path

