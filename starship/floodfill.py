import dataclasses
from platform import mac_ver
from re import M
import cv2
import numpy as np

class FloodFill:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.image = self.getImage()
        cv2.imshow("map", self.image)
        #self.edgeDetectionTest()
        cv2.waitKey(1)
    
    # Turn the OccupancyGrid into an OpenCV image/numpy array
    # Reads the array as signed 16-bit values to preserve -1 values,
    # then replaces -1 with 128 before converting the image to unsigned bytes.
    # PGM image files also use a mid-grey for unexplored space, treating near-zero
    # as free space, near-255 as an obstacle and the middle as unknown.
    def getImage(self):
        data = np.asarray(self.map.data, dtype=np.int16).reshape(self.map.info.height, self.map.info.width)
        data[data < 0] = 128
        data = data.astype(np.uint8)
        return data
    
    def edgeDetectionTest(self):
        edges = cv2.Canny(self.image,0,128)
        cv2.imshow("edges", edges)