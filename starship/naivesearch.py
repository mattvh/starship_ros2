from geometry_msgs.msg import Pose

# Naive frontier detection using only OccupancyGrid data and no OpenCV processing
class NaiveSearch:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.frontierPoints = []
    
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
    
    def toPoses(self):
        return self.frontierPoints
    
    def targetPoints(self):
        return self.frontierPoints