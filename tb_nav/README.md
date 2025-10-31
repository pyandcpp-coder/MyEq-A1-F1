# TurtleBot3 Navigation Package

I built this package to handle all the navigation capabilities for my TurtleBot3 robot. It provides SLAM mapping, localization, and autonomous navigation using Nav2.

## Overview

This package provides three main functionalities:

1. **SLAM** - Create maps of unknown environments using `slam_toolbox`
2. **Localization** - Localize the robot on existing maps using AMCL
3. **Navigation** - Autonomous navigation using the Nav2 stack

## Package Structure

```
tb_nav/
├── CMakeLists.txt                          # Build configuration
├── package.xml                             # Package dependencies
├── README.md                               # This file
├── config/
│   ├── mapper_params_online_async.yaml    # SLAM Toolbox configuration
│   └── nav2_params.yaml                   # Nav2 stack parameters
├── launch/
│   ├── bringup_nav.launch.py              # Main navigation launch (with localization)
│   ├── nav2_launch_include_file.py        # Nav2 servers launch file
│   └── online_async_slam.launch.py        # SLAM mapping launch
└── maps/
    ├── my_world_map.pgm                   # Map image file
    └── my_world_map.yaml                  # Map metadata
```

## How It Works

### Launch Files

I created three launch files for different purposes:

#### 1. **bringup_nav.launch.py** - Full Navigation Stack

This is my main navigation launch file. It starts:

- **Simulation** - Launches `tb_sim` package (Gazebo + RVIZ)
- **Map Server** - Loads the pre-built map (`my_world_map.yaml`)
- **AMCL** - Adaptive Monte Carlo Localization for robot pose estimation
- **Static TF** - Publishes map→odom transform
- **Nav2 Stack** - All navigation servers (controller, planner, behavior, etc.)
- **Lifecycle Manager** - Manages node lifecycle for map_server and AMCL

This is what I use when the map is already created and I want to navigate.

#### 2. **online_async_slam.launch.py** - SLAM Mapping

This launch file is for creating new maps:

- **SLAM Toolbox** - Asynchronous SLAM node for mapping
- Uses parameters from `mapper_params_online_async.yaml`
- Builds the map in real-time as the robot explores

I use this when I need to map a new environment.

#### 3. **nav2_launch_include_file.py** - Nav2 Servers Only

This is an include file that launches all the Nav2 servers:

- **Controller Server** - Tracks the path and controls robot velocity
- **Smoother Server** - Smooths planned paths
- **Planner Server** - Computes global paths from A to B
- **Behavior Server** - Executes recovery behaviors (spin, backup, etc.)
- **BT Navigator** - Behavior tree coordinator
- **Waypoint Follower** - Follows waypoint sequences
- **Velocity Smoother** - Smooths velocity commands to the robot
- **Lifecycle Manager** - Manages all navigation nodes

### Configuration Files

#### mapper_params_online_async.yaml

Contains SLAM Toolbox parameters for mapping:
- Map resolution and update rates
- Scan matching parameters
- Loop closure settings
- Optimization parameters

#### nav2_params.yaml

Contains all Nav2 parameters:
- **Controller settings** - DWB controller parameters, speed limits, trajectory scoring
- **Planner settings** - NavFn/SmacPlanner configuration
- **AMCL settings** - Particle filter parameters, laser model
- **Recovery behaviors** - Spin, backup, wait parameters
- **Costmap parameters** - Global and local costmap settings (inflation, obstacles)

### Map Files

The `maps/` directory contains my pre-built map:

- **my_world_map.pgm** - Grayscale image (white=free, black=occupied, gray=unknown)
- **my_world_map.yaml** - Map metadata:
  - Resolution: 0.05 meters per pixel
  - Origin: [-5.72, -4.78, 0] in the map frame
  - Thresholds for occupied/free space

## Dependencies

Required packages (specified in `package.xml`):

- **nav2_bringup** - Nav2 navigation stack
- **slam_toolbox** - SLAM mapping
- **nav2_map_server** - Map loading and serving
- **tf2_ros** - Transform library
- **launch_ros** & **launch** - Launch system

## Building

Build the package from your workspace:

```bash
cd ~/myeq_ws
colcon build --packages-select tb_nav
source install/setup.bash
```

## Usage

### Creating a New Map (SLAM)

If you need to create a new map:

1. **Start the simulation** (if not already running):
   ```bash
   ros2 launch tb_sim start_world_and_robot.launch.py
   ```

2. **Launch SLAM**:
   ```bash
   ros2 launch tb_nav online_async_slam.launch.py
   ```

3. **Drive the robot** to explore the environment:
   - Use keyboard teleoperation: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
   - Or use joystick control
   - Or use RVIZ "Nav2 Goal" to navigate

4. **Save the map** when done:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/myeq_ws/src/tb_nav/maps/my_world_map
   ```

### Navigation with Existing Map

To navigate using the pre-built map:

1. **Launch the full navigation stack**:
   ```bash
   ros2 launch tb_nav bringup_nav.launch.py
   ```

   This single command starts:
   - Gazebo simulation
   - RVIZ visualization
   - Map server with `my_world_map`
   - AMCL localization
   - All Nav2 navigation servers

2. **Set initial pose** in RVIZ:
   - Click "2D Pose Estimate" button
   - Click and drag on the map where the robot actually is
   - AMCL will localize the robot

3. **Send navigation goals**:
   - **Option A**: Use RVIZ "Nav2 Goal" button to click destinations
   - **Option B**: Use `waypoint_nav_pkg` GUI for predefined waypoints
   - **Option C**: Publish goals programmatically via `/navigate_to_pose` action

### Using Only SLAM (Standalone)

To run SLAM without full navigation:

```bash
ros2 launch tb_nav online_async_slam.launch.py use_sim_time:=true
```

Note: You'll need to launch the simulation separately.

## How Navigation Works

### Localization Flow

1. **Map Server** loads the map from `my_world_map.yaml`
2. **AMCL** uses laser scans to estimate robot pose on the map
3. **Static TF** publishes the map→odom transform
4. Robot pose is continuously updated based on sensor data

### Navigation Flow

1. User sends a goal pose (via RVIZ, waypoint GUI, or programmatically)
2. **BT Navigator** receives the goal and orchestrates the navigation
3. **Planner Server** computes a global path from current pose to goal
4. **Controller Server** generates velocity commands to follow the path
5. **Velocity Smoother** smooths the commands for safer motion
6. Robot executes the commands and reaches the goal
7. If stuck, **Behavior Server** executes recovery behaviors

### Costmaps

The system maintains two costmaps:

- **Global Costmap** - Built from the static map, used for global planning
- **Local Costmap** - Built from recent sensor data, used for local obstacle avoidance

Both use:
- Inflation layer (makes robot avoid obstacles with safety margin)
- Obstacle layer (marks obstacles from laser scans)

## Key Topics

### Subscribed Topics
- `/scan` - Laser scan data for localization and obstacle detection
- `/odom` - Odometry from robot wheels

### Published Topics
- `/cmd_vel` - Final velocity commands to the robot
- `/cmd_vel_nav` - Pre-smoothed velocity commands
- `/plan` - Global path visualization
- `/local_plan` - Local trajectory visualization
- `/map` - Occupancy grid map
- `/particle_cloud` - AMCL particle filter visualization

### Action Servers
- `/navigate_to_pose` - Main navigation action (used by waypoint_nav_pkg)
- `/follow_waypoints` - Follow sequence of waypoints

## Configuration Tips

### Adjusting Robot Speed

Edit `nav2_params.yaml`, find `controller_server` section:
```yaml
max_vel_x: 0.26      # Maximum forward velocity (m/s)
max_vel_theta: 1.0   # Maximum angular velocity (rad/s)
min_vel_x: 0.0       # Minimum forward velocity
```

### Tuning AMCL Localization

Edit `nav2_params.yaml`, find `amcl` section:
```yaml
max_particles: 2000  # More particles = better accuracy but slower
min_particles: 500   # Fewer particles = faster but less accurate
```

### Changing Path Planner

Edit `nav2_params.yaml`, find `planner_server`:
```yaml
plugin: "nav2_navfn_planner/NavfnPlanner"  # Or use SmacPlanner
```

## Troubleshooting

### Robot Doesn't Localize
- Make sure you set the initial pose with "2D Pose Estimate" in RVIZ
- Check if laser scan matches the map (visible in RVIZ)
- Verify map is loaded: `ros2 topic echo /map --once`
- Increase AMCL particles if needed

### Navigation Fails or Robot Gets Stuck
- Check if global plan is visible in RVIZ (blue line)
- Verify local costmap shows obstacles correctly
- Look for recovery behaviors being triggered
- Check terminal for error messages from Nav2 servers

### Map Server Fails
- Verify map files exist in `maps/` directory
- Check `my_world_map.yaml` points to correct `.pgm` file
- Ensure map origin and resolution are correct

### Transform Errors (TF)
- Verify static transform is published: `ros2 run tf2_ros tf2_echo map odom`
- Check if AMCL is running: `ros2 node list | grep amcl`
- Ensure `use_sim_time` is set correctly (true for simulation)

### Lifecycle Manager Issues
- Check node names in lifecycle manager match actual node names
- Look for "Failed to activate" messages in terminal
- Verify all Nav2 servers are running: `ros2 node list`

## Integration with Other Packages

This package works with:

- **tb_sim** - Provides the simulation environment (launched automatically by `bringup_nav.launch.py`)
- **waypoint_nav_pkg** - Uses this package's navigation stack via `/navigate_to_pose` action

Typical workflow:
1. *(Optional)* Create map using `online_async_slam.launch.py`
2. Launch navigation with `bringup_nav.launch.py`
3. Use `waypoint_nav_pkg` for waypoint-based navigation

## Notes

- All nodes use `use_sim_time: true` for Gazebo synchronization
- AMCL requires an initial pose estimate to start localizing
- The velocity smoother remaps topics to ensure smooth command flow
- Nav2 uses behavior trees for complex navigation logic
- Map resolution is 0.05m (5cm per pixel)

