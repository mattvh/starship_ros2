import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from visualization_msgs.msg import MarkerArray

from starship.frontierfinder import FrontierFinder


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.map = OccupancyGrid()
        self.registerSubscribers()
        self.registerPublishers()
    
    def registerSubscribers(self):
        map_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)
        self.create_subscription(OccupancyGrid(), '/map', self.handleOccupancyGrid, map_qos)
    
    def registerPublishers(self):
        marker_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)
        self.markerPub = self.create_publisher(MarkerArray, 'frontiers', qos_profile=QoSProfile(depth=10))
    
    def handleOccupancyGrid(self, data):
        self.map = data
        FrontierFinder(self)


def main(args=None):
    rclpy.init(args=args)
    print('Hi from starship!')
    explorer = Explorer()
    rclpy.spin(explorer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
