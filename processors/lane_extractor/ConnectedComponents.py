import numpy as np
import cv2
from UnionFind import *
import operator
import cProfile
import dis

class ConnectedComponents:
    
    def __init__(self, image):
        self.image = image

    def getPoints(self, sets):
        roots = set(sets.parents[x] for x in sets)
        roots_weights = [(r, sets.weights[r]) for r in roots]
        mrw = max(roots_weights, key=operator.itemgetter(1))
        points = [p for p in sets.parents if sets.parents[p] == mrw[0]]
        return points

    def doFindConnectedComponents(self, binary, marked):
        sets = UnionFind()

        count = 0
        on = 255
        for y in xrange(binary.shape[0]):
            for x in xrange(binary.shape[1]):
                if binary[y, x] != 255:
                    continue
                # Count the "on" pixels, for debugging
                count += 1
                xyname = sets[y, x]
                # TODO: there's probably a much better way to do this
                adjnames = set()
                #this is hackery and shouldn't work at all
                if x - 1 >= 0 and binary[y, x - 1] is 255:
                    adjnames.add(sets[y, x - 1])
                    if y-1 >= 0 and binary[y - 1, x - 1] is 255:
                        adjnames.add(sets[y - 1, x - 1])

                if y - 1 >= 0 and binary[y - 1, x] is 255:
                    adjnames.add(sets[y - 1, x])

                # For efficiency, no need to union if its the same root
                while xyname in adjnames:
                    adjnames.remove(xyname)
                if adjnames:
#                    print "Merging", xyname, "with", adjnames
                    sets.union(xyname, *adjnames)
        points = self.getPoints(sets)


  #      print count, len(roots), mrw

        # Mark the largest connected component in the image
        for p in points:
            marked[p] = (255, 0, 0)
#                ret, rect = cv2.floodFill(image, mask, (x,y), label_count, flags=4)

#                blob = []
#                print rect
#                for i in xrange(rect[1], rect[1]+rect[3]):
#
#                    for j in xrange(rect[0], rect[0]+rect[2]):
#                        if image[y,x] != label_count:
#                            continue;
#                        #blob.push_back(cv::Point(j,i));
#     
#    #            blobs.push_back(blob);
#     
#                label_count+=1
#        return image
#
    def findConnectedComponents(self):
        
        self.image = self.image[0:][300:]
        ccMarked = np.copy(self.image)
        imgray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,0,255,0)
     
        thresh = cv2.medianBlur(thresh, 3)
        image = self.doFindConnectedComponents(thresh, ccMarked) #np.zeros(thresh.shape, dtype=np.uint8)
        
        cv2.imshow("thresh", thresh)
        cv2.imshow("image", ccMarked)

        cv2.imwrite("ccmarked.jpg", ccMarked)

    def showTest(self):
        self.findConnectedComponents()
        #cv2.drawContours(self.image, self.getPoly(), 0, (255, 0, 0), 3)
        #cv2.imshow("Results", self.image)
#        cv2.waitKey(0)

if __name__ == "__main__":
    image = cv2.imread("testimage.png")
    #p = ConnectedComponents(image)
    #p.showTest()
    cProfile.run('ConnectedComponents(image).showTest()')
    dis.dis(ConnectedComponents.doFindConnectedComponents)
