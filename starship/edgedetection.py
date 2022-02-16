import cv2
import numpy as np
from geometry_msgs.msg import Pose

class EdgeDetection:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.occupiedThresh = 50
        self.image = self.getImage()
        self.edges = self.edgeDetection()
        self.debugImage()
    

    def debugImage(self):
        if self.node.debug:
            cv2.imshow("map", self.image)
            cv2.imshow("edges", self.edges)
            cv2.waitKey(1)
    

    # Turn the OccupancyGrid into an OpenCV image/numpy array
    # Reads the array as signed 16-bit values to preserve -1 values,
    # then replaces -1 with 128 before converting the image to unsigned bytes.
    # PGM image files also use a mid-grey for unexplored space, treating near-zero
    # as free space, near-255 as an obstacle and the middle as unknown.
    # While doing this, the map is cleaned up to create harder edges.
    # The only values in the map image should be 255 for free space, 128 for unknown
    # and 0 for an obstacle.
    def getImage(self):
        data = np.asarray(self.map.data, dtype=np.int16).reshape(self.map.info.height, self.map.info.width)
        mask_obs = np.logical_and(data > self.occupiedThresh, data < 255)
        mask_free = np.logical_and(data >= 0, data < self.occupiedThresh)
        data[mask_free] = 255
        data[mask_obs] = 0
        data[data < 0] = 128
        data = data.astype(np.uint8)
        return data


    # Find the frontier pixels with edge detection.
    # Canny Edge detection is used to find edge lines, which are then filtered
    # with a mask of the obstacles in the map to remove non-frontier pixels.
    # i.e. we want pixels that neighbor the unexplored gray color pixels and
    # not black pixels.
    def edgeDetection(self):
        #Canny Edge Detection
        edges = cv2.Canny(self.image,10,200)
        #Dilated obstacle mask, to cover unwanted edges
        obs = self.image.copy()
        obs[obs != 0] = 255
        obs = cv2.bitwise_not(obs)
        kernel = np.ones((3,3), np.uint8)
        obs = cv2.dilate(obs, kernel, iterations=1)
        #Bitwise and the mask and the edges to remove the non-frontier edges
        edges = cv2.bitwise_and(cv2.bitwise_not(obs), edges)
        edges = self.denoise(edges)
        return edges
    

    # Remove stray pixels that don't have adjacent edge pixels
    def denoise(self, img):
        height, width = img.shape
        for y in range(0, height-1):
            for x in range(0, width-1):
                if img[y,x] == 255:
                    adjPix = 0
                    for p in self.adjacentPixels(x, y):
                        if img[p[1], p[0]] == 255:
                            adjPix += 1
                    if adjPix < 1:
                        img[y,x] = 0
        return img
    

    # Returns the eight pixels surrounding a given pixel
    def adjacentPixels(self, x, y):
        return [(x+1,y), (x-1,y), (x,y+1), (x,y-1), (x+1,y+1), (x-1,y-1), (x+1,y-1), (x-1,y+1)]
    

    # Turn the edge lines into larger solids with dilate(), then find the centroids.
    # This returns better points to drive to than simply picking the nearest frontier point.
    def targetPoints(self):
        targets = []
        # Dilate the edges to create "solids" to operate on
        kernel = np.ones((2,2), np.uint8)
        img = cv2.dilate(self.edges, kernel, iterations=1)
        # Find the centroids of the newly created shapes and save the coordinates
        contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 3, (255, 0, 0), -1)
            targets.append(self.pixelToPose(cX, cY))
        return targets

    
    # Convert the edge pixels to ROS Poses
    def toPoses(self):
        poses = []
        height, width = self.edges.shape
        res = self.map.info.resolution
        for mapY in range(0, height-1):
            for mapX in range(0, width-1):
                if self.edges[mapY, mapX] != 0:
                    pose = self.pixelToPose(mapX, mapY)
                    poses.append(pose)
        return poses
    

    # Utility method to convert a given pixel x,y coordinate to a ROS Pose
    def pixelToPose(self, pixelX, pixelY):
        res = self.map.info.resolution
        x = (pixelX * res) + self.map.info.origin.position.x + (res/2)
        y = (pixelY * res) + self.map.info.origin.position.y + (res/2)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose