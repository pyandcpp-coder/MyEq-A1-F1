#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Empty
from action_msgs.msg import GoalStatus
import time

# Format: 'name': [x, y, z, qx, qy, qz, qw]
WAYPOINTS = {
    'station_a': [-3.25, 3.69, 0.0, 0.0, 0.0, -0.893, 0.449],
    'station_b': [-4.28, 0.07, 0.0, 0.0, 0.0, -0.540, 0.841],
    'station_c': [-3.79, -2.95, 0.0, 0.0, 0.0, 0.010, 0.999],
    'station_d': [3.99, -3.45, 0.0, 0.0, 0.0, 0.708, 0.706],
    'station_e': [3.50, -0.07, 0.0, 0.0, 0.0, 0.483, 0.875],
    'station_f': [3.95, 3.31, 0.0, 0.0, 0.0, -0.997, 0.071],
    'home': [0.019, -0.149, 0.0, 0.0, 0.0, 0.650, 0.759],
    'docking': [0.166, -4.30, 0.0, 0.0, 0.0, 0.703, 0.710]
}

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('NavigateToPose action server not available, waiting...')
            
        self.create_subscription(String, 'go_to_single_waypoint', self.single_waypoint_callback, 10)
        self.create_subscription(String, 'go_to_waypoint_sequence', self.sequence_callback, 10)
        self.create_subscription(Empty, 'cancel_navigation', self.cancel_callback, 10)
        
        self.status_publisher = self.create_publisher(String, 'navigation_status', 10)
        
        self.goal_handle = None
        self.is_navigating = False
        
        self.get_logger().info('Waypoint Manager is ready.')

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(message)

    def single_waypoint_callback(self, msg):
        waypoint_name = msg.data
        if self.is_navigating:
            self.publish_status(f'Already navigating. Ignoring request to {waypoint_name}.')
            return
            
        if waypoint_name not in WAYPOINTS:
            self.publish_status(f'Invalid waypoint: {waypoint_name}')
            return
            
        self.is_navigating = True
        waypoints_to_visit = [waypoint_name, 'home']
        self.execute_navigation_sequence(waypoints_to_visit)

    def sequence_callback(self, msg):
        waypoint_names = [name.strip() for name in msg.data.split(',') if name.strip()]
        if self.is_navigating:
            self.publish_status('Already navigating. Ignoring sequence request.')
            return
            
        valid_waypoints = []
        for name in waypoint_names:
            if name in WAYPOINTS:
                valid_waypoints.append(name)
            else:
                self.publish_status(f'Warning: Skipping invalid waypoint: {name}')
        
        if not valid_waypoints:
            self.publish_status('No valid waypoints in sequence.')
            return
            
        self.is_navigating = True
        valid_waypoints.append('home')
        self.execute_navigation_sequence(valid_waypoints)

    def cancel_callback(self, msg):
        if not self.is_navigating or not self.goal_handle:
            self.publish_status('No active navigation to cancel.')
            return
            
        self.publish_status('Cancelling navigation...')
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        self.is_navigating = False
        self.publish_status('Navigation cancelled.')

    def create_pose_stamped(self, waypoint_name):
        coords = WAYPOINTS[waypoint_name]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = coords[0]
        pose.pose.position.y = coords[1]
        pose.pose.position.z = coords[2]
        pose.pose.orientation.x = coords[3]
        pose.pose.orientation.y = coords[4]
        pose.pose.orientation.z = coords[5]
        pose.pose.orientation.w = coords[6]
        return pose

    def execute_navigation_sequence(self, waypoints):
        if not waypoints:
            self.is_navigating = False
            self.publish_status('Sequence complete. Returning to idle.')
            return
        
        waypoint_name = waypoints.pop(0)
        self.publish_status(f'Navigating to: {waypoint_name}...')
        
        goal_pose = self.create_pose_stamped(waypoint_name)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        send_goal_future.add_done_callback(
            lambda future: self.goal_accepted_callback(future, waypoints, waypoint_name)
        )

    def goal_accepted_callback(self, future, waypoints, waypoint_name):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.publish_status(f'Goal for {waypoint_name} was rejected.')
            self.is_navigating = False
            return
            
        self.publish_status(f'Goal for {waypoint_name} accepted. Waiting for result...')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.get_result_callback(future, waypoints, waypoint_name)
        )

    def get_result_callback(self, future, waypoints, waypoint_name):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(f'Successfully reached: {waypoint_name}')
            time.sleep(1.0)
            self.execute_navigation_sequence(waypoints)
        else:
            self.is_navigating = False
            self.publish_status(f'Navigation failed for {waypoint_name}. Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()