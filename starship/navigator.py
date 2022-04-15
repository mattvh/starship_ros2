from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.duration import Duration

class Navigator:
    def __init__(self, node):
        self.node = node
        self.goalHandle = None
        self.resultFuture = None
        self.feedback = None
        self.status = None
        self.goalPose = None
        node.get_logger().info("Initializing Navigator.")
        self.navPoseClient = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.waitUntilReady()
    
    # Public method to begin driving to a supplied pose
    def driveTo(self, pose):
        if self.isNavComplete():
            self.checkIfNavigationAborted()
            self.goToPose(pose)
        while not self.isNavComplete():
            self.cancelGoalIfUnreachable()
            pass
    
    # Send the goal to the Navigation2 action server
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
        self.node.get_logger().info(f"Set Navigation goal to {pose.position.x}, {pose.position.y}.")
        self.goalPose = pose
        self.resultFuture = self.goalHandle.get_result_async()
        return True
    
    # Check if navigation is complete and spin the future so the robot will drive
    # Method adaptedz from Navigation2's example BasicNavigator class
    def isNavComplete(self):
        if not self.resultFuture:
            return True
        rclpy.spin_until_future_complete(self.node, self.resultFuture, timeout_sec=1.00)
        if self.resultFuture.result():
            self.status = self.resultFuture.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False
        return True
    
    def cancelGoalIfUnreachable(self):
        if not self.feedback:
            return
        if Duration.from_msg(self.feedback.navigation_time) > Duration(seconds=180.0):
            self.node.get_logger().info("Not reaching goal. Canceling.")
            if self.resultFuture:
                future = self.goalHandle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, future)
    
    def checkIfNavigationAborted(self):
        if self.status != None and self.status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Navigation unsuccessful. Getting new target.")
            self.node.getNextTarget(skipTarget=self.node.target)
    
    # Callback to store current feedback from nav2
    def handleUpdates(self, msg):
        self.feedback = msg.feedback
        return
    
    # Set the initial pose of the robot to its estimated position from SLAM
    def setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.node.robotPose.pose
        msg.header.frame_id = self.node.robotPose.header.frame_id
        msg.header.stamp = self.node.robotPose.header.stamp
        self.node.initial_pose_pub.publish(msg)
        return
    
    # Wait until the Navigation2 action server is ready
    def waitUntilReady(self):
        self.node.get_logger().info("Waiting for Navigation2 action server...")
        while not self.navPoseClient.wait_for_server(timeout_sec=1.0):
            rclpy.spin_once(self.node, timeout_sec=1.0)
        self.setInitialPose()
        self.node.get_logger().info("Navigation ready.")
        return
    
    # Convert a Pose to a PoseStamped
    def poseToPoseStamped(self, pose):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rclpy.time.Time().to_msg()
        p.pose.position = pose.position
        p.pose.orientation = pose.orientation
        return p