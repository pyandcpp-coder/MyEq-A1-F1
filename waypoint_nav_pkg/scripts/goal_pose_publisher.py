#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        
        # Create QoS profile matching Nav2
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_profile)
        
        # Subscribe to initial pose estimate (2D Pose Estimate button in RViz)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/initialpose',
            self.pose_callback,
            10)
        
        self.get_logger().info('Goal Pose Publisher started!')
        self.get_logger().info('Use "2D Pose Estimate" button in RViz to set waypoints with orientation.')
        self.get_logger().info('Publishing to /goal_pose')

    def pose_callback(self, msg):
        # Republish as goal_pose
        goal_msg = PoseStamped()
        goal_msg.header = msg.header
        goal_msg.header.frame_id = 'map'
        goal_msg.pose = msg.pose
        
        self.publisher.publish(goal_msg)
        
        # Log the waypoint
        pos = msg.pose.position
        orient = msg.pose.orientation
        
        # Calculate yaw from quaternion for easier reading
        import math
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info('--- New Waypoint Published! ---')
        self.get_logger().info(f"  position:")
        self.get_logger().info(f"    x: {pos.x}")
        self.get_logger().info(f"    y: {pos.y}")
        self.get_logger().info(f"    z: {pos.z}")
        self.get_logger().info(f"  orientation:")
        self.get_logger().info(f"    x: {orient.x}")
        self.get_logger().info(f"    y: {orient.y}")
        self.get_logger().info(f"    z: {orient.z}")
        self.get_logger().info(f"    w: {orient.w}")
        self.get_logger().info(f"  yaw (radians): {yaw:.4f}")
        self.get_logger().info(f"  yaw (degrees): {math.degrees(yaw):.2f}")
        self.get_logger().info('---------------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
