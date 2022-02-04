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
        cv2.imshow("map", self.image)
        cv2.imshow("edges", self.edges)
        cv2.waitKey(1)
    

    # Turn the OccupancyGrid into an OpenCV image/numpy array
    # Reads the array as signed 16-bit values to preserve -1 values,
    # then replaces -1 with 128 before converting the image to unsigned bytes.
    # PGM image files also use a mid-grey for unexplored space, treating near-zero
    # as free space, near-255 as an obstacle and the middle as unknown.
    # While doing this, the map is cleaned up to create harder edges.
    # The only values in the map image should be 0 for free space, 128 for unknown
    # and 255 for an obstacle.
    def getImage(self):
        data = np.asarray(self.map.data, dtype=np.int16).reshape(self.map.info.height, self.map.info.width)
        mask_obs = np.logical_and(data > self.occupiedThresh, data < 255)
        mask_free = np.logical_and(data >= 0, data < self.occupiedThresh)
        data[mask_free] = 255
        data[mask_obs] = 0
        data[data < 0] = 128
        data = data.astype(np.uint8)
        return data


    # Find the frontier pixels with edge detection
    def edgeDetection(self):
        edges = cv2.Canny(self.image,10,200)
        # Iterate the resulting image and remove non-frontier edges by setting
        # the pixel color to black if the corresponding pixel in the raw map image
        # has a neighbor pixel with the unexplored gray color.
        height, width = edges.shape
        for y in range(0, height-1):
            for x in range(0, width-1):
                frontPix = False
                for p in self.adjacentPixels(x, y):
                    if p[0] < 0 or p[0] > width or p[1] < 0 or p[1] > height:
                        continue
                    if self.image[p[1], p[0]] == 0:
                        edges[y, x] = 0
                        break
                    if edges[y, x] != 0 and self.image[p[1], p[0]] == 128:
                        frontPix = True #white edge pixel and unexplored map pixel
                if not frontPix:
                    edges[y, x] = 0
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
    

    def adjacentPixels(self, x, y):
        return [(x+1,y), (x-1,y), (x,y+1), (x,y-1), (x+1,y+1), (x-1,y-1), (x+1,y-1), (x-1,y+1)]


    def targetPoints(self):
        targets = []
        #lines = cv2.HoughLinesP(self.edges, rho=1, theta=1*np.pi/180, threshold=16, minLineLength=25, maxLineGap=250)
        #for line in lines:
        #    y1, x1, y2, x2 = line[0]
        #    targets.append(self.pixelToPose(y1, x1))
        #    targets.append(self.pixelToPose(y2, x2))
        #contours, hierarchy = cv2.findContours(self.edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #indices = np.where(self.edges != [0])
        #coordinates = zip(indices[0], indices[1])
        #for c in coordinates:
        #    print(c)
            #targets.append(self.pixelToPose(c[1], c[0]))
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