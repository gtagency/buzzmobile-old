import numpy as np
import cv2

class PolyFinder:
    
    def __init__(self, image):
        self.image = image

    def getPoly(self):
        
        self.image = self.image[0:][300:]
        imgray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,0,255,0)
        thresh = cv2.medianBlur(thresh, 3)
        cv2.imshow("thresh", thresh)
        height = thresh.shape[0]
        width = thresh.shape[1]
        
        visited = set()
        stack = []
        stack.append((height / 2, (width / 2) + 25))
        while stack :
            point = stack.pop()
            if point in visited:
                continue
            if thresh[point[0] - 1][point[1]] != 0:
                stack.append((point[0], point[1] - 1))

            if thresh[point[0] + 1][point[1]] != 0:
                stack.append((point[0], point[1] + 1))

            if thresh[point[0]][point[1] - 1] != 0:
                stack.append((point[0] - 1, point[1]))

            if thresh[point[0]][point[1] + 1] != 0:
                stack.append((point[0] + 1, point[1]))
            visited.add(point)
        marked = np.copy(self.image)
        for p in visited :
            marked[p] = (255,0,0)
        cv2.imshow("Marked", marked)
    def showTest(self):
        self.getPoly()
        #cv2.drawContours(self.image, self.getPoly(), 0, (255, 0, 0), 3)
        #cv2.imshow("Results", self.image)
        cv2.waitKey(0)

if __name__ == "__main__":
    image = cv2.imread("testimage.png")
    p = PolyFinder(image)
    p.showTest()
