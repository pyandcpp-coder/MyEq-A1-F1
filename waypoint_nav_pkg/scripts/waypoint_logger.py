#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        # Create QoS profile for Nav2 goal pose (transient local)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to goal_pose with transient local QoS
        self.subscription1 = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_profile)
        
        # Subscribe to clicked_point with default QoS
        self.subscription2 = self.create_subscription(
            PoseStamped,
            '/clicked_point',
            self.goal_callback,
            10)
        
        # Subscribe to PointStamped version of clicked_point
        self.subscription3 = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10)
        
        # Subscribe to initialpose (2D Pose Estimate button) - uses PoseWithCovarianceStamped!
        self.subscription4 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waypoint Logger Node Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('To log waypoints with ORIENTATION:')
        self.get_logger().info('  1. Click "2D Pose Estimate" button in RViz')
        self.get_logger().info('  2. Click and drag on the map to set position and direction')
        self.get_logger().info('')
        self.get_logger().info('To log waypoints WITHOUT orientation:')
        self.get_logger().info('  - Click "Publish Point" button in RViz')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for waypoints...')

    def goal_callback(self, msg):
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('WAYPOINT RECEIVED (with orientation)')
        self.get_logger().info('=' * 60)
        
        pose = msg.pose
        pos = pose.position
        orient = pose.orientation
        
        # Calculate yaw from quaternion
        import math
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info('Position:')
        self.get_logger().info(f'  x: {pos.x}')
        self.get_logger().info(f'  y: {pos.y}')
        self.get_logger().info(f'  z: {pos.z}')
        self.get_logger().info('Orientation:')
        self.get_logger().info(f'  x: {orient.x}')
        self.get_logger().info(f'  y: {orient.y}')
        self.get_logger().info(f'  z: {orient.z}')
        self.get_logger().info(f'  w: {orient.w}')
        self.get_logger().info(f'Yaw: {yaw:.4f} rad ({math.degrees(yaw):.2f} deg)')
        self.get_logger().info('=' * 60)
    
    def initial_pose_callback(self, msg):
        # This is for logging when 2D Pose Estimate is used
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('WAYPOINT (from 2D Pose Estimate)')
        self.get_logger().info('=' * 60)
        
        pose = msg.pose.pose  # Note: PoseWithCovarianceStamped has pose.pose
        pos = pose.position
        orient = pose.orientation
        
        # Calculate yaw from quaternion
        import math
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info('Position:')
        self.get_logger().info(f'  x: {pos.x}')
        self.get_logger().info(f'  y: {pos.y}')
        self.get_logger().info(f'  z: {pos.z}')
        self.get_logger().info('Orientation:')
        self.get_logger().info(f'  x: {orient.x}')
        self.get_logger().info(f'  y: {orient.y}')
        self.get_logger().info(f'  z: {orient.z}')
        self.get_logger().info(f'  w: {orient.w}')
        self.get_logger().info(f'Yaw: {yaw:.4f} rad ({math.degrees(yaw):.2f} deg)')
        self.get_logger().info('=' * 60)
    
    def point_callback(self, msg):
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('WAYPOINT (no orientation)')
        self.get_logger().info('=' * 60)
        
        point = msg.point
        
        self.get_logger().info('Position:')
        self.get_logger().info(f'  x: {point.x}')
        self.get_logger().info(f'  y: {point.y}')
        self.get_logger().info(f'  z: {point.z}')
        self.get_logger().info('Orientation: (default)')
        self.get_logger().info(f'  x: 0.0')
        self.get_logger().info(f'  y: 0.0')
        self.get_logger().info(f'  z: 0.0')
        self.get_logger().info(f'  w: 1.0')
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
