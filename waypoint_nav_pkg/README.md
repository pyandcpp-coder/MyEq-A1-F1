# Waypoint Navigation Package

I built this ROS 2 package to provide an intuitive GUI-based waypoint navigation system for my robot. The package allows me to send the robot to predefined waypoints either individually or in a sequence, and everything returns home automatically after completion.

## Overview

This package consists of three main components working together:

1. **Waypoint Manager** - The backend node that handles navigation logic and communicates with Nav2
2. **Waypoint GUI** - The tkinter-based graphical interface for selecting and controlling navigation
3. **Launch File** - Brings up the entire navigation stack including Nav2 and both nodes

## How It Works

### Architecture

The system uses a publisher-subscriber pattern with ROS 2 topics to coordinate between the GUI and the navigation manager:

- **GUI publishes** commands to topics that the manager subscribes to
- **Manager executes** navigation goals through Nav2's action server
- **Manager publishes** status updates back to the GUI

### Waypoint Definitions

I defined 8 hardcoded waypoints in the system (in `waypoint_manager.py`):

- **station_a** through **station_f**: Six stations positioned around the environment
- **home**: The default starting/return position at (0.019, -0.149)
- **docking**: A special docking station at (0.166, -4.30)

Each waypoint stores a full 3D pose: `[x, y, z, qx, qy, qz, qw]` where the quaternion represents the robot's orientation.

### Navigation Modes

I implemented two navigation modes:

#### 1. Single Waypoint Mode
- Select one or more waypoints (only the first is used)
- Robot navigates to the selected waypoint
- Automatically returns to home position
- Button: "Go to First Selected (and Home)"

#### 2. Sequence Mode
- Select multiple waypoints in any order
- Robot visits each waypoint sequentially
- Automatically returns to home at the end
- Button: "Run Full Sequence (and Home)"

Both modes automatically append 'home' to the navigation queue, so I never have to manually send the robot home.

## Components Breakdown

### 1. Waypoint Manager (`waypoint_manager.py`)

This is the brain of the operation. Here's what it does:

**Initialization:**
- Creates an action client for Nav2's `NavigateToPose` action
- Subscribes to three command topics:
  - `go_to_single_waypoint` - For single waypoint navigation
  - `go_to_waypoint_sequence` - For sequential navigation
  - `cancel_navigation` - For emergency stops
- Publishes status updates to `navigation_status`

**Key Functions:**

- `single_waypoint_callback()` - Handles single waypoint requests, always adds 'home' to the list
- `sequence_callback()` - Parses comma-separated waypoint names, validates them, and adds 'home'
- `execute_navigation_sequence()` - Recursively sends goals to Nav2 one at a time
- `create_pose_stamped()` - Converts waypoint coordinates to ROS 2 PoseStamped messages
- `cancel_callback()` - Immediately cancels the current navigation goal

**Navigation Flow:**
1. Receives waypoint request
2. Sets `is_navigating` flag to prevent overlapping goals
3. Sends goal to Nav2 action server
4. Waits for goal acceptance
5. Waits for goal completion
6. On success, moves to next waypoint in sequence (recursive call)
7. On failure or completion, sets `is_navigating` to False

### 2. Waypoint GUI (`waypoint_gui.py`)

My tkinter-based interface that makes controlling the robot easy:

**UI Components:**

- **Waypoint Checkboxes**: 8 checkboxes arranged in a grid (4 columns) for waypoint selection
- **Control Buttons**: 
  - "Go to First Selected (and Home)" - Single mode
  - "Run Full Sequence (and Home)" - Sequence mode  
  - "CANCEL Navigation" - Emergency stop
- **Status Display**: Shows real-time feedback from the manager node

**Threading:**
- GUI runs on the main thread for proper tkinter operation
- ROS 2 spinning happens on a daemon thread to handle callbacks
- Status updates from ROS are thread-safe with tkinter variables

**Publishers:**
- `go_to_single_waypoint` - Publishes String with waypoint name
- `go_to_waypoint_sequence` - Publishes String with comma-separated names
- `cancel_navigation` - Publishes Empty message

**Key Functions:**

- `get_selected_waypoints()` - Extracts checked waypoint names from the UI
- `go_to_single()` - Takes first selected waypoint, publishes it
- `go_to_sequence()` - Joins all selected waypoints with commas, publishes
- `status_callback()` - Updates GUI status label when manager publishes updates

### 3. Launch File (`final_waypoint_gui.launch.py`)

I set this up to start everything with one command:

**What It Launches:**

1. **Nav2 Stack** - Includes the entire navigation system from `tb_nav` package
   - AMCL localization
   - Controller server
   - Planner server
   - Behavior server
   - All required lifecycle managers

2. **Waypoint Manager Node** - The backend navigation coordinator

3. **Waypoint GUI Node** - The user interface with `emulate_tty=True` for proper GUI display

## Topics and Communication

### Published Topics (by GUI):
- `/go_to_single_waypoint` (std_msgs/String) - Single waypoint command
- `/go_to_waypoint_sequence` (std_msgs/String) - Sequence command (comma-separated)
- `/cancel_navigation` (std_msgs/Empty) - Cancel command

### Subscribed Topics (by GUI):
- `/navigation_status` (std_msgs/String) - Status updates from manager

### Action Client (by Manager):
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose) - Nav2 navigation action

## Dependencies

I made sure to include all necessary dependencies in `package.xml`:

- **rclpy** - ROS 2 Python client library
- **geometry_msgs** - For PoseStamped messages
- **std_msgs** - For String and Empty messages
- **nav2_msgs** - For NavigateToPose action
- **ament_cmake** & **ament_cmake_python** - Build tools

Additionally, you need:
- **tkinter** - Python GUI library (usually pre-installed with Python)
- **Nav2 stack** - Full navigation framework
- **tb_nav package** - My custom navigation configuration package

## Building and Installation

### 1. Build the Package

Navigate to your workspace and build:

```bash
cd ~/myeq_ws
colcon build --packages-select waypoint_nav_pkg
source install/setup.bash
```

### 2. Make Scripts Executable (if needed)

```bash
chmod +x ~/myeq_ws/src/waypoint_nav_pkg/scripts/waypoint_manager.py
chmod +x ~/myeq_ws/src/waypoint_nav_pkg/scripts/waypoint_gui.py
```

## Usage

### Starting the System

Launch everything with a single command:

```bash
ros2 launch waypoint_nav_pkg final_waypoint_gui.launch.py
```

This will:
- Start the Nav2 navigation stack
- Launch the waypoint manager node
- Open the GUI window

### Using the GUI

1. **Wait for Initialization**: Allow a few seconds for Nav2 to fully start up and the action server to become available

2. **Select Waypoints**: Click the checkboxes for the waypoint(s) you want to visit

3. **Choose Navigation Mode**:
   - Click "Go to First Selected (and Home)" for single waypoint navigation
   - Click "Run Full Sequence (and Home)" to visit all selected waypoints in order

4. **Monitor Status**: Watch the status display at the bottom for real-time feedback

5. **Cancel if Needed**: Click "CANCEL Navigation" to immediately stop the robot

### Example Workflows

**Visit Station A and Return:**
1. Check "station_a"
2. Click "Go to First Selected (and Home)"
3. Robot navigates to station_a, then returns home

**Patrol All Stations:**
1. Check all stations: station_a, station_b, station_c, station_d, station_e, station_f
2. Click "Run Full Sequence (and Home)"
3. Robot visits each station in order, then returns home

**Emergency Stop:**
- Click "CANCEL Navigation" at any time
- Current goal is cancelled immediately
- Robot stops moving

## Important Notes

### Waypoint Coordinates

The waypoint coordinates in `WAYPOINTS` dictionary must match your actual map. I measured these positions in the map frame using RVIZ:

- Use RVIZ's "2D Pose Estimate" tool to get coordinates
- Update the `WAYPOINTS` dictionary in `waypoint_manager.py` with your positions
- Rebuild the package after modifications

### Navigation Safety

- The system prevents overlapping navigation requests using the `is_navigating` flag
- If you try to send a new goal while navigating, it will be rejected
- Always ensure your map is properly loaded before starting navigation
- Verify the robot is localized correctly in RVIZ before sending waypoints

### Status Messages

The manager publishes detailed status messages:
- "Navigating to: [waypoint_name]..." - Goal sent
- "Goal for [waypoint_name] accepted..." - Nav2 accepted the goal
- "Successfully reached: [waypoint_name]" - Waypoint reached
- "Navigation failed for [waypoint_name]..." - Something went wrong
- "Navigation cancelled." - User cancelled navigation

## Troubleshooting

### GUI Doesn't Appear
- Check if tkinter is installed: `python3 -c "import tkinter"`
- Ensure `emulate_tty=True` is in the launch file
- Verify the node is running: `ros2 node list | grep waypoint_gui`

### Robot Doesn't Move
- Confirm Nav2 action server is running: `ros2 action list | grep navigate_to_pose`
- Check if the map is loaded in RVIZ
- Verify the robot is properly localized
- Look at the terminal output for error messages from the manager

### Invalid Waypoint Errors
- Double-check waypoint names in `WAYPOINT_NAMES` (GUI) match `WAYPOINTS` (manager)
- Both lists must be synchronized

### Navigation Fails
- Check costmap for obstacles blocking the path
- Verify waypoint coordinates are within the map boundaries
- Ensure the goal pose orientation (quaternion) is valid
- Look at Nav2 logs for detailed error information

## Future Improvements

Ideas I might implement later:

- **Dynamic Waypoints**: Add/remove waypoints through the GUI instead of hardcoding
- **Save/Load**: Store waypoint configurations in YAML files
- **Visual Feedback**: Show current target on RVIZ markers
- **Pause/Resume**: Ability to pause sequence and resume later
- **Waypoint Ordering**: Drag-and-drop to reorder sequence in GUI
- **Status History**: Keep a log of completed waypoints
- **Retry Logic**: Automatically retry failed waypoints

## File Structure

```
waypoint_nav_pkg/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── README.md                   # This file
├── launch/
│   └── final_waypoint_gui.launch.py  # Main launch file
└── scripts/
    ├── waypoint_manager.py     # Backend navigation coordinator
    └── waypoint_gui.py         # Frontend GUI interface
```

