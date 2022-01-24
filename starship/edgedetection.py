import cv2
import numpy as np
from geometry_msgs.msg import Pose

class EdgeDetection:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.image = self.getImage()
        self.edges = self.edgeDetection()
        self.debugImage()
    

    def debugImage(self):
        cv2.imshow("map", self.image)
        cv2.imshow("edges", self.edges)
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


    # Find the frontier pixels with edge detection
    def edgeDetection(self):
        # Blur the image and perform Canny Edge Detection with a tight threshold
        blurred = cv2.GaussianBlur(self.image, (5, 5), 0)
        edges = cv2.Canny(blurred,10,200)
        # Iterate the resulting image and remove non-frontier edges by setting
        # the pixel color to black if the corresponding pixel in the raw map image
        # has a neighbor pixel with the unexplored gray color.
        height, width = edges.shape
        for y in range(0, height-1):
            for x in range(0, width-1):
                adj = [(x+1,y), (x-1,y), (x,y+1), (x,y-1)]
                frontPix = False
                for p in adj:
                    if p[0] < 0 or p[0] > width or p[1] < 0 or p[1] > height:
                        continue
                    if edges[y, x] != 0 and self.image[p[1], p[0]] == 128:
                        frontPix = True #white edge pixel and unexplored map pixel
                if not frontPix:
                    edges[p[1], p[0]] = 0
        return edges

    
    # Convert the edge pixels to ROS Poses
    def toPoses(self):
        poses = []
        height, width = self.edges.shape
        res = self.map.info.resolution
        for mapY in range(0, height-1):
            for mapX in range(0, width-1):
                if self.edges[mapY, mapX] != 0:
                    x = (mapX * res) + self.map.info.origin.position.x + (res/2)
                    y = (mapY * res) + self.map.info.origin.position.y + (res/2)
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = 0.0
                    pose.orientation.w = 1.0
                    poses.append(pose)
        return poses