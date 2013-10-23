

class RoadNetwork(object):

    def __init__(self, networkFilePath):
        """
            Constructs a road network from a file on disk
        """
        with open(networkFilePath, 'r') as file:
            
