# ROS 2 Waypoint Navigation Project

This project implements a full navigation system for a TurtleBot3 robot in a custom Gazebo simulation. I created a simple Tkinter GUI to select predefined waypoints, and a ROS 2 "manager" node handles the logic to send the robot to these locations sequentially using the Nav2 stack.

The primary goal was to design and implement a system where a TurtleBot3 robot in a custom Gazebo map autonomously navigates to user-selected waypoints chosen via a simple GUI.

## How to Run the Project

1. **Build the Workspace**: From the root of the workspace (myeq_ws), build all packages:
   ```bash
   colcon build
   ```

2. **Source and Launch**: In a single terminal, source the workspace and run the main launch file. This one command starts Gazebo, RViz, Nav2, and the custom GUI.
   ```bash
   source install/setup.bash
   ros2 launch waypoint_nav_pkg final_waypoint_gui.launch.py
   ```

3. **Localize and Navigate**:
   - Once RViz loads, use the "2D Pose Estimate" tool to set the robot's initial position on the map.
   - Use the Waypoint Navigation GUI window to select one or more waypoints.
   - Click "Go to First Selected" or "Run Full Sequence" to begin navigation.

## Project Structure

My workspace is organized into three main packages:

### tb_sim

- **Purpose**: Handles the simulation environment.
- **Key Files**:
  - `worlds/final.sdf`: The custom Gazebo world I built for navigation.
  - `rviz/myeq_rviz_config.rviz`: The custom RViz configuration, which is set up to publish goals to `/goal_pose`.
  - `launch/start_world_and_robot.launch.py`: Launches Gazebo and spawns the TurtleBot3.

### tb_nav

- **Purpose**: Configures and launches the full Nav2 stack.
- **Key Files**:
  - `maps/my_world_map.yaml`: The map file generated from the Gazebo world.
  - `config/nav2_params.yaml`: All parameters for AMCL, BT Navigator, controllers, and costmaps.
  - `launch/bringup_nav.launch.py`: The main launch file that starts the tb_sim and all Nav2 components.
  - `launch/nav2_launch_include_file.py`: A helper file that organizes all the individual Nav2 lifecycle nodes.

### waypoint_nav_pkg

- **Purpose**: Contains all my custom Python logic to fulfill the project requirements.
- **Key Files**:
  - `scripts/waypoint_manager.py`: The "brain" of the operation.
  - `scripts/waypoint_gui.py`: The Tkinter user interface.
  - `launch/final_waypoint_gui.launch.py`: The top-level launch file for the entire project.

## Core Logic & System Architecture

The system's logic is designed to be simple and robust, decoupled into two main nodes.

### GUI (waypoint_gui.py)

- Provides a simple Tkinter window with checkboxes for all waypoints and buttons for "Go to Single", "Run Sequence", and "Cancel".
- It publishes user requests as simple `std_msgs/String` messages to topics like `go_to_single_waypoint` and `go_to_waypoint_sequence`.
- It subscribes to the `/navigation_status` topic and displays any string it receives in its status label.

### Manager (waypoint_manager.py)

- This node contains a hard-coded dictionary of all the waypoint coordinates I collected.
- It subscribes to the topics from the GUI.
- When it receives a request, it becomes an action client for Nav2's `/navigate_to_pose` action server.
- It converts the waypoint names (e.g., "Station A") into a full `PoseStamped` message and sends it as a goal to Nav2.
- It manages the navigation flow, waiting for the action to complete.
- **Single Waypoint**: It navigates to the selected goal, and upon success, it automatically sends a new goal to return to "Home".
- **Sequence**: It iterates through the list of selected waypoints, sending them one by one, and only sends the next goal after the previous one succeeds. After the final waypoint, it navigates to "Home".
- It publishes human-readable status updates (e.g., "Navigating to: Station A", "Successfully reached: Station A") to the `/navigation_status` topic for the GUI to display.

## Key Challenges & Solutions

During development, I encountered two major configuration issues that prevented navigation from working.

### 1. The RViz "Nav2 Goal" Button Did Nothing

- **Problem**: After setting up the stack, clicking the "Nav2 Goal" button in RViz would print a message, but the robot would not move. My `waypoint_logger.py` script also showed no messages on `/goal_pose`.

- **Investigation**: I discovered this happens because the default Nav2 configuration has two different ways of sending goals:
  - Publishing to the `/goal_pose` topic.
  - Calling the `/navigate_to_pose` action server directly.

  My RViz (`myeq_rviz_config.rviz`) was configured to publish to `/goal_pose`, but my Nav2 `bt_navigator` node was not listening to it.

- **Solution**: I edited `tb_nav/config/nav2_params.yaml` and added a single line to the `bt_navigator` parameters. This explicitly tells the `bt_navigator` to subscribe to the `/goal_pose` topic for goal requests:
  ```yaml
  bt_navigator:
    ros__parameters:
      ...
      goal_topic: /goal_pose
  ```

### 2. The Entire Navigation Stack Failed to Start

- **Problem**: After fixing the `goal_topic` issue, my navigation stack (costmaps, planners, etc.) was still dead. The launch log showed an ERROR from the `collision_monitor` node and a fatal error from the `lifecycle_manager_navigation`.

- **Investigation**: The log showed `[collision_monitor]: Error while getting parameters: parameter 'observation_sources' is not initialized`. This meant the `nav2_launch_include_file.py` was launching the `collision_monitor` node, but I had not provided any parameters for it in `nav2_params.yaml`. This single node's failure caused the `lifecycle_manager` to abort bringing up all other navigation nodes.

- **Solution**: Since the `collision_monitor` was not a core requirement for this project, I disabled it. In `tb_nav/launch/nav2_launch_include_file.py`, I removed `collision_monitor` from the `node_names` list in the `lifecycle_manager_navigation` node definition, which allowed the rest of the navigation stack to launch correctly.

## Project Author

- **Name**: Yash Tiwari
- **Email**: yashtiwari9182@gmail.com