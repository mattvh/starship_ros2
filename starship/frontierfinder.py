from starship.edgedetection import EdgeDetection
from starship.naivesearch import NaiveSearch
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import math, sys

class FrontierFinder:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.frontierPoints = []
        self.minDriveUnits = 10
        detector = None
        if self.node.altSearch:
            detector = NaiveSearch(self.node)
        else:
            detector = EdgeDetection(self.node)
        self.frontierPoints = detector.toPoses()
        self.targetPoints = detector.targetPoints()
        self.publishMarkers()
    
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
    
    # Public method to get the next target to drive to
    def getNextTarget(self):
        return self.getNearestPose(self.targetPoints)
    
    # If there are no more frontier points, we've explored the whole area
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
        for i in range(2, len(self.targetPoints)):
            now = self.node.get_clock().now()
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
            if self.targetPoints[i] == self.node.target:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker.pose.orientation.w = 1.0
            marker.pose.position = self.targetPoints[i].position
            markers.markers.append(marker)
        self.node.markerPub.publish(markers)