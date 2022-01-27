from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.action import ActionClient
import rclpy

class Navigator:
    def __init__(self, node):
        self.node = node
        node.get_logger().info("Initializing Navigator.")
        self.nav_pose_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.waitUntilReady()
        self.timer = self.node.create_timer(1.0, self.navTimer)
    
    def navTimer(self):
        #what this should do:
        #Check if currently driving to a target. If not, ask the frontier finder for the
        #next target. Do driveTo() on the target. Else, do nothing as we are driving.
        if self.node.target:
            self.driveTo(self.node.target)
    
    def driveTo(self, pose):
        while not self.nav_pose_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info("'NavigateToPose' action server not available, waiting...")
        goal = NavigateToPose.Goal()
        goal.pose = self.poseToPoseStamped(pose)
        future = self.nav_pose_client.send_goal_async(goal, self.handleUpdates)
        rclpy.spin_until_future_complete(self.node, future)
        while not self.driving(future):
            pass
    
    def driving(self, future):
        if not future:
            return False
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
        if self.future.result():
            self.status = self.result_future.result().status
            if self.status != 1:
                return False
        else:
            return True
    
    def handleUpdates(self, msg):
        print(msg.feedback)
    
    def setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.node.robotPose.pose
        msg.header.frame_id = self.node.robotPose.header.frame_id
        msg.header.stamp = self.node.robotPose.header.stamp
        self.node.initial_pose_pub.publish(msg)
        return
    
    def waitUntilReady(self):
        self.node.get_logger().info("Waiting for Navigation2 action server...")
        while not self.nav_pose_client.wait_for_server(timeout_sec=1.0):
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