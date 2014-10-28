import cv2
import numpy as np
import sys
import detector_functions as df
class DetectorTest:
    def __init__(self, pointCloud):
        self.pointCloud = pointCloud

    def findObstacles(self):
        #may not be necesarry
        points = df.polar2cart(points)
        groups = df.group_points(points)
        hulls = [df.graham_scan(g) for g in groups]
        image = np.zeros((800,800), np.dtype_uint8)
        for h in hulls:
            for p in h:
                image[p[1],p[0]] = 255
        cv.imshow("hulls",image)
if __name__ = "__main__":
    points = readPointCloud(sys.argv[2]):
    dt = DetectorTest(
