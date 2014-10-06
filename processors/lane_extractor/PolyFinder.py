import numpy as np
import cv2

class PolyFinder:
    
    def __init__(self, image):
        self.image = image

    def getPoly(self):
        
        self.image = self.image[0:][300:]
        imgray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,0,255,0)
        
        #cv2.bitwise_not(thresh)
        height = thresh.shape[0]
        width = thresh.shape[1]
        tempImage = np.copy(thresh)
        fillPoint = None
        for x in range(height - 1, 0, -1):
            currPixel = thresh[x][width / 2]
            if currPixel.all()  != 0:
                 fillPoint = ((width/ 2), x)
                 print fillPoint
                 break
        dim = (height + 2, width + 2)
        mask = np.zeros(dim, dtype=np.uint8)
        
        #Produces nothing if the fill point is used
        #If (0, 0) is used it fills in noise
        cv2.floodFill(thresh, mask, (0, 0), 255)
        cv2.imshow("filledImage", thresh)

        #removes most noise from the thresholded image
        noiseRemoved = cv2.bitwise_xor(thresh, tempImage)
        
        #Dilates in order to remove more noise
        cv2.dilate(noiseRemoved, np.ones((4,4), dtype=np.uint8), noiseRemoved, (-1, -1), 1)
        
        cv2.imshow("f", noiseRemoved)

    def showTest(self):
        self.getPoly()
        #cv2.drawContours(self.image, self.getPoly(), 0, (255, 0, 0), 3)
        #cv2.imshow("Results", self.image)
        cv2.waitKey(0)

if __name__ == "__main__":
    image = cv2.imread("testimage.png")
    p = PolyFinder(image)
    p.showTest()
