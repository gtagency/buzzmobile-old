
import io
import json
from pprint import pprint
from itertools import *
from distance import *
from search import  *

class Node(object):
    def __init__(self, latitude, longitude):
        self.latitude = float(latitude)
        self.longitude = float(longitude)

    def __repr__(self):
        return "(lat: %f, lon: %f)" % (self.latitude, self.longitude)

#    def __str__(self):
 #       self.__repr__()
         
class RoadNetwork(object):

    def __init__(self, networkFilePath):
        """
            Constructs a road network from a file on disk
        """
        with open(networkFilePath, 'r') as file:
            data = json.loads(file.read())
        
        self.nodes = []
        self.edges = []

        for p in data["points"]:
            loc = p["location"]
            parts = loc.split(",")
            self.nodes.append(Node(parts[0], parts[1]))

        for f in data["edges"]["forward"]:
            start = f["start"] - 1
            end   = f["end"] - 1
            self.edges.append((self.nodes[start], self.nodes[end]))
        
        for r in data["edges"]["reverse"]:
            start = r["start"] - 1
            end   = r["end"] - 1
            self.edges.append((self.nodes[start], self.nodes[end]))
        
      #  pprint(data)

    # TODO: add distance threshold (e.g. if we're 100 miles from the nearest node, we're probably not on that node
    def findClosestNode(self, latitude, longitude):
        pairs = [(euclidean(n.latitude, n.longitude, latitude, longitude), n) for n in self.nodes]
    #    pprint(pairs)
        return min(pairs)[1]
    
    def findClosestNodes(self, latitude, longitude, num = 1):
        pairs = [(euclidean(n.latitude, n.longitude, latitude, longitude), n) for n in self.nodes]
     #   pprint(pairs)
        nodes = []

        for i in xrange(num):
            minPair = min(pairs)
            nodes.append(minPair[1])
            nodes.remove(minPair)

        return nodes

    def getOutboundEdges(self, node):
        return [edge for edge in self.edges if edge[0] == node];

    def findPathFromPosition(self, myLatitude, myLongitude, destLatitude, destLongitude):
        closest = self.findClosestNode(myLatitude, myLongitude)
        dest    = self.findClosestNode(destLatitude, destLongitude)
        return AStarSearch(self).path(closest, dest)

def loadDefault():
    return RoadNetwork('../data/parade_route_graph.json')
