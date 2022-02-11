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
        self.altSearch = False
        self.debug = False
        self.registerParameters()
        self.registerSubscribers()
        self.registerPublishers()
        self.waitForInitialPose()
        self.poseTimer = self.create_timer(0.1, self.checkRobotPose)
        self.navigator = Navigator(self)
    
    # Register configuration parameters this node accepts
    def registerParameters(self):
        self.declare_parameter('drive', True)
        self.declare_parameter('altsearch', False)
        self.declare_parameter('debug', False)
        self.altSearch = self.get_parameter('altsearch').get_parameter_value().bool_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

    # Register ROS topic subscribers
    def registerSubscribers(self):
        # Subscribe to Occupancy Grid from SLAM node
        map_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)
        self.create_subscription(OccupancyGrid(), '/map', self.handleOccupancyGrid, map_qos)
        # Create TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    # Register ROS topic publishers
    def registerPublishers(self):
        # Publish frontiers for RViz2 to display
        self.markerPub = self.create_publisher(MarkerArray, 'frontiers', qos_profile=QoSProfile(depth=1))
        # Publish the robot's starting position, as determined by SLAM, to initialize Navigation2
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    
    # Save the map when the topic updates
    # and select the next target to drive to
    def handleOccupancyGrid(self, data):
        self.map = data
        self.get_logger().info("Scanning frontiers.")
        self.getNextTarget()
    
    # Run the frontier finder, generating a list of frontier points
    # and select the next target for the robot to drive to.
    def getNextTarget(self):
        finder = FrontierFinder(self)
        if finder.noMoreFrontierPoints():
            self.get_logger().info("No more frontiers.")
        if len(finder.targetPoints) > 0:
            newTarget = finder.getNextTarget()
            if newTarget is not None:
                oldTarget = self.target
                self.target = newTarget
                if oldTarget != self.target:
                    self.get_logger().info(f"Set target to {self.target.position.x}, {self.target.position.y}")

    # Timer callback to check the TF tree for the robot's current pose
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
    
    # Don't initialize Navigation2 until the robot's starting pose is determined.
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
