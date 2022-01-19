from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion

class FrontierFinder:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.frontierPoints = []
        self.search()
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
                    if self.map.data[p] != -1: #adjacent point is explored
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
    
    # Publish a MarkerArray to visualize frontier points in RViz2
    def publishMarkers(self):
        coords = [(0.0,0.0), (0.0, 0.5), (0.0, 1.0), (0.5, 0.0), (1.0, 0.0)]
        markers = MarkerArray()
        for i in range(0, len(self.frontierPoints)):
            now = self.node.get_clock().now()
            marker = Marker()
            marker.id = i
            marker.ns = 'Frontier'
            marker.header.frame_id = "map"
            marker.header.stamp = now.to_msg()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color = ColorRGBA(r=0.5, g=0., b=0.5, a=0.8)
            marker.pose.orientation.w = 1.0
            marker.pose.position = self.frontierPoints[i].position
            markers.markers.append(marker)
        self.node.markerPub.publish(markers)