from turtle import pos
from starship.edgedetection import EdgeDetection
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion
import math, sys

class FrontierFinder:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.frontierPoints = []
        self.minDriveUnits = 10
        #self.search()
        edgeDetection = EdgeDetection(self.node)
        self.frontierPoints = edgeDetection.toPoses()
        self.targetPoints = []
        nearest = self.getNearestPose(self.frontierPoints)
        if nearest is not None:
            self.targetPoints.append(nearest) 
        self.publishMarkers()
    
    # Search for frontier points
    def search(self):
        for i in range(0, len(self.map.data)):
            if self.isFrontierPoint(i):
                self.frontierPoints.append(self.indexToPose(i))
    
    # Determine if an index in the map is a frontier point, meaning it is unexplored and adjacent
    # to area that has already been expored.
    # OccupancyGrid uses -1 for unexplored area, 0 for free, 1-100 for propability of obstacle
    def isFrontierPoint(self, i):
        if self.map.data[i] == -1: #point is unexplored
            adj = [i+1, i-1, i+self.map.info.width, i-self.map.info.width] #adjacent indices
            for p in adj:
                if p >= 0 and p < len(self.map.data):
                    if self.map.data[p] != -1 and self.map.data[p] < 5: #adjacent point is explored
                        return True
        return False
    
    # Convert a linear OccupancyGrid index to an (x,y) tuple
    def indexToXY(self, i):
        x = i // self.map.info.width
        y = i % self.map.info.height
        return (x, y)
    
    # Convert an OccupancyGrid array index to a world Pose object
    def indexToPose(self, i):
        mapXY = self.indexToXY(i)
        res = self.map.info.resolution
        x = (mapXY[0] * res) + self.map.info.origin.position.x + (res/2)
        y = (mapXY[1] * res) + self.map.info.origin.position.y + (res/2)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose
    
    # Find the nearest pose in the list to the robot's position
    def getNearestPose(self, poses):
        if self.node.robotPose == None:
            return None
        start = self.node.robotPose.pose
        nearest = None
        nearestDist = sys.float_info.max
        for pose in poses:
            dist = math.hypot(start.position.x-pose.position.x, start.position.y-pose.position.y)
            if (dist < nearestDist) and (dist > (self.map.info.resolution * self.minDriveUnits)):
                nearest = pose
                nearestDist = dist
        return nearest
    
    def noMoreFrontierPoints(self):
        return len(self.frontierPoints) < 1
    
    # Publish a MarkerArray to visualize frontier points in RViz2
    def publishMarkers(self):
        markers = MarkerArray()
        # Clear Existing Markers
        marker = Marker()
        marker.id = 0
        marker.ns = 'Frontier'
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        marker = Marker()
        marker.id = 1
        marker.ns = 'FrontierTarget'
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        # Frontier Points
        for i in range(2, len(self.frontierPoints)):
            now = self.node.get_clock().now()
            marker = Marker()
            marker.id = i
            marker.ns = 'Frontier'
            marker.header.frame_id = "map"
            marker.header.stamp = now.to_msg()
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = self.map.info.resolution
            marker.scale.y = self.map.info.resolution
            marker.scale.z = self.map.info.resolution
            marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0)
            marker.pose.orientation.w = 1.0
            marker.pose.position = self.frontierPoints[i].position
            markers.markers.append(marker)
        # Target Points
        for i in range(len(self.targetPoints)):
            marker = Marker()
            marker.id = i
            marker.ns = 'FrontierTarget'
            marker.header.frame_id = "map"
            marker.header.stamp = now.to_msg()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = self.map.info.resolution * 2
            marker.scale.y = self.map.info.resolution * 2
            marker.scale.z = self.map.info.resolution * 2
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker.pose.orientation.w = 1.0
            marker.pose.position = self.targetPoints[i].position
            markers.markers.append(marker)
        self.node.markerPub.publish(markers)