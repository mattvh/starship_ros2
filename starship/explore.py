import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from visualization_msgs.msg import MarkerArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf2_ros import TransformException

from starship.frontierfinder import FrontierFinder
from starship.navigator import Navigator


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.map = OccupancyGrid()
        self.robotPose = None
        self.target = None
        self.registerParameters()
        self.registerSubscribers()
        self.registerPublishers()
        self.waitForInitialPose()
        self.poseTimer = self.create_timer(0.1, self.checkRobotPose)
        self.navigator = Navigator(self)
    
    def registerParameters(self):
        self.declare_parameter('drive', True)

    def registerSubscribers(self):
        map_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)
        self.create_subscription(OccupancyGrid(), '/map', self.handleOccupancyGrid, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def registerPublishers(self):
        self.markerPub = self.create_publisher(MarkerArray, 'frontiers', qos_profile=QoSProfile(depth=1))
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    
    def handleOccupancyGrid(self, data):
        self.map = data
        self.get_logger().info("Scanning frontier.")
        self.getNextTarget()
    
    def getNextTarget(self):
        finder = FrontierFinder(self)
        if finder.noMoreFrontierPoints():
            print("No more frontiers.")
            exit()
        if len(finder.targetPoints) > 0:
            newTarget = finder.getNextTarget()
            if newTarget is not None:
                oldTarget = self.target
                self.target = newTarget
                if oldTarget != self.target:
                    self.get_logger().info(f"Set target to {self.target.position.x}, {self.target.position.y}")

    def checkRobotPose(self):
        from_frame = 'base_link'
        to_frame = 'map'
        try:
            now = rclpy.time.Time()
            dur = Duration()
            dur.sec = 40
            dur.nsec = 0
            t = self.tf_buffer.lookup_transform(to_frame, from_frame, now, dur)
            pose = PoseStamped()
            pose.header.frame_id = to_frame
            pose.header.stamp = rclpy.time.Time().to_msg()
            point = Point()
            point.x = t.transform.translation.x
            point.y = t.transform.translation.y
            point.z = t.transform.translation.z
            pose.pose.position = point
            pose.pose.orientation = t.transform.rotation
            self.robotPose = pose
        except TransformException as ex:
            self.robotPose = None
        return
    
    def waitForInitialPose(self):
        self.get_logger().info("Waiting for initial pose from TF...")
        while not self.robotPose:
            self.checkRobotPose()
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Initial pose found.")


def main(args=None):
    rclpy.init(args=args)
    print('Starship initializing.')
    explorer = Explorer()
    drive = explorer.get_parameter('drive').get_parameter_value().bool_value
    while rclpy.ok():
        rclpy.spin_once(explorer)
        if explorer.target and drive:
            explorer.navigator.driveTo(explorer.target)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
