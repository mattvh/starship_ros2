from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import rclpy

class Navigator:
    def __init__(self, node):
        self.node = node
        self.goalHandle = None
        self.resultFuture = None
        self.feedback = None
        node.get_logger().info("Initializing Navigator.")
        self.navPoseClient = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.waitUntilReady()
    
    # Public method to begin driving to a supplied pose
    def driveTo(self, pose):
        self.node.get_logger().info("driveTo")
        self.goToPose(pose)
        self.node.get_logger().info("driving loop start")
        while not self.isNavComplete():
            self.node.get_logger().info("loop driving")
            pass
    
    # Send the initial goal to the Navigation2 action server
    # Method adapted from Navigation2's example BasicNavigator class
    def goToPose(self, pose):
        while not self.navPoseClient.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info("NavigateToPose server not available, waiting...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.poseToPoseStamped(pose)
        send_goal_future = self.navPoseClient.send_goal_async(goal_msg, self.handleUpdates)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        self.goalHandle = send_goal_future.result()
        if not self.goalHandle.accepted:
            self.node.get_logger().error("Goal rejected.")
            return False
        self.resultFuture = self.goalHandle.get_result_async()
        return True
    
    # Check if navigation is complete and spin the future so the robot will drive
    # Method adapted from Navigation2's example BasicNavigator class
    def isNavComplete(self):
        if not self.resultFuture:
            return True
        rclpy.spin_until_future_complete(self.node, self.resultFuture, timeout_sec=0.10)
        if self.resultFuture.result():
            self.status = self.resultFuture.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False
        return True
    
    def handleUpdates(self, msg):
        self.feedback = msg.feedback
        return
    
    def setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.node.robotPose.pose
        msg.header.frame_id = self.node.robotPose.header.frame_id
        msg.header.stamp = self.node.robotPose.header.stamp
        self.node.initial_pose_pub.publish(msg)
        return
    
    def waitUntilReady(self):
        self.node.get_logger().info("Waiting for Navigation2 action server...")
        while not self.navPoseClient.wait_for_server(timeout_sec=1.0):
            rclpy.spin_once(self.node, timeout_sec=1.0)
        self.setInitialPose()
        self.node.get_logger().info("Navigation ready.")
        return
    
    def poseToPoseStamped(self, pose):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rclpy.time.Time().to_msg()
        p.pose.position = pose.position
        p.pose.orientation = pose.orientation
        return p