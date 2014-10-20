import cv2
import numpy as np
import random

class Planner:
    def __init__(self,image):
        self.image = image
    
    def pickPoint(self, lastAdded):
        choice = random.randint(1,5)
        distance = random.randint(1, 10)
        if choice is 1:
             return (lastAdded[0] - distance, lastAdded[1])
        if choice is 2:
             return (lastAdded[0] - (distance/2), lastAdded[1] - (distance/2))
        if choice is 3:
             return (lastAdded[0] - (distance/2), lastAdded[1] + (distance/2))
        if choice is 4:
             return (lastAdded[0], lastAdded[1] - distance)
        if choice is 5:
             return (lastAdded[0], lastAdded[1] + distance)
    
    def findGoal(self):
        return (0 , 400)
    
    def getStart(self):
        return (475, 400)
   
    def findDistance(self, point1, point2):
        return (((point1[1] - point2[1]) ** 2) + ((point1[0] - point2[0]) ** 2)) ** 0.5
    
    def findNearest(self, points, nextPoint):
        minimum = self.findDistance(points[0], nextPoint)
        minpoint = points[0]
        for i in (range(len(points) - 1)):
            currDistance = findDistance(points[i+1], nextPoint)
            if currDistance < minimum :
                minimum = currDistance
                minpoint = points[i+1]
        return minpoint

    def pathExists(self, point):
        topright = (point[0] - 2, point[1] - 2)
        white = 0
        black = 0
        for y in range(4) :
            for x in range(4):
                if self.image[(topright[0] + y, topright[1] + x)] is 0:
                    black = black + 1
                else:
                    white = white + 1
        return (black / float(white)) > 0.6

    def rrt(self):
        epsilon = 4
        goal = self.findGoal()
        start = self.getStart()
        tree = dict()
        tree[start] = []
        lastAdded = start
        while goal not in tree.values():
            nextPoint = self.pickPoint(lastAdded)
            if self.image[nextPoint] is 0 or nextPoint in tree.values():
                continue
            nearestPoint = self.findNearest(tree.keys(), nextPoint)
            if self.pathExists(nextPoint):
                tree[nearestPoint].append(nextPoint)
                tree[nextPoint] = []
                if findDistance(nextPoint, goal) < epsilon:
                    tree[nextPoint].append(goal)
            else:
                continue
        return bfs(tree) 
    
    def bfs(self, tree):
        visited = []
        stack = [self.getStart()]
        while stack:
            currPoint = stack.pop()
            if currPoint in visited:
                continue
            stack = stack + tree[currPoint]
            visited.append(currPoint)
        return visited

    def drawPath(self):
        path = self.rrt()
        for p in path:
            self.image[p] = 0
        cv2.imshow(self.image)

if __name__ == "__main__":
    image = cv2.imread("testimage.png")
    p = Planner(image)
    p.drawPath()
