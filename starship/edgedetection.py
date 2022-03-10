import cv2
import math
import numpy as np
from geometry_msgs.msg import Pose

class EdgeDetection:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.occupiedThresh = 50
        self.image = self.getImage()
        self.edges = self.edgeDetection()
        self.inflationMap = self.obstacleInflation()
        self.debugImage()
    

    def debugImage(self):
        if self.node.debug:
            cv2.imshow("map", self.image)
            cv2.imshow("edges", self.edges)
            if self.inflationMap is not None:
                cv2.imshow("inflation", self.inflationMap)
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
        #Bitwise AND the mask with the edges to remove the non-frontier edges
        edges = cv2.bitwise_and(cv2.bitwise_not(obs), edges)
        edges = self.removeStrays(edges)
        return edges
    

    # Remove stray pixels that don't have adjacent edge pixels
    def removeStrays(self, img):
        img_adj = self.createNeighborArray(img)
        adjPix = img_adj.sum(axis=0)
        img_cp = img.copy()
        img_cp[adjPix < 1] = 0
        return img_cp
    

    # Create neighbor array for removeStrays()
    def createNeighborArray(self, img):
        img_adj = np.zeros((8, *img.shape), dtype=img.dtype) # 8-nbr-hood array
        count = 0
        for r_shift in [-1, 0, 1]:
            for c_shift in [-1, 0, 1]:
                if r_shift == 0 and c_shift == 0:
                    continue
                else:
                    img_adj[count,
                            max(r_shift, 0):(r_shift if r_shift < 0 else None),
                            max(c_shift, 0):(c_shift if c_shift < 0 else None)
                    ] = img[max(-r_shift, 0):(-r_shift if -r_shift < 0 else None),
                            max(-c_shift, 0):(-c_shift if -c_shift < 0 else None)]
                    count += 1
        return img_adj
    

    # Handle the inflation layer on Navigation2's global costmap.
    # This is used to avoid picking points that Nav2 will not be able to drive to.
    def obstacleInflation(self):
        if self.node.navCostmap is None:
            return None
        costmap = self.node.navCostmap
        img = np.asarray(costmap.data, dtype=np.int16).reshape(costmap.metadata.size_y, costmap.metadata.size_x)
        mask_obs = np.logical_and(img > 252, img < 255)
        mask_free = np.logical_and(img >= 0, img < 252)
        img[mask_free] = 255
        img[mask_obs] = 0
        return img.astype(np.uint8)
    

    # Returns true if a given coordinate is within the Navigation2 inflation layer.
    def pointInInflationLayer(self, x, y):
        return (self.inflationMap is not None) and (self.inflationMap[y, x] == 0)
    

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
            if (self.pointInInflationLayer(cX, cY)):
                continue
            targets.append(self.pixelToPose(cX, cY))
        return targets

    
    # Convert the edge pixels to ROS Poses
    def toPoses(self):
        poses = []
        indices = np.where(self.edges != [0])
        coords = zip(indices[0], indices[1])
        for pixel in coords:
            pose = self.pixelToPose(pixel[1], pixel[0])
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