# TurtleBot3 Simulation Package

I created this package to handle all the simulation setup for my TurtleBot3 robot in Gazebo. It launches the complete simulation environment with the robot, world, and visualization tools.

## Overview

This package provides a single launch file that starts everything I need for simulation:
- Gazebo simulator with a custom world
- TurtleBot3 robot model spawned in the world
- Robot state publisher for TF transforms
- RVIZ2 with my custom configuration

## Package Structure

```
tb_sim/
├── CMakeLists.txt                          # Build configuration
├── package.xml                             # Package dependencies
├── README.md                               # This file
├── launch/
│   └── start_world_and_robot.launch.py    # Main launch file
├── rviz/
│   └── myeq_rviz_config.rviz             # Custom RVIZ configuration
└── worlds/
    ├── final.sdf                          # Main simulation world (currently used)
    ├── my_world.sdf                       # Alternative world layout
    ├── test_world.sdf                     # Testing world
    ├── model.sdf                          # World model definition
    └── model.config                       # Model metadata
```

## How It Works

### Launch File (`start_world_and_robot.launch.py`)

The launch file orchestrates the entire simulation startup:

1. **Environment Variable**: Reads `TURTLEBOT3_MODEL` (defaults to 'burger')
2. **Gazebo Launch**: Starts Gazebo with the `final.sdf` world file
3. **Robot State Publisher**: Publishes robot transforms and joint states
4. **Spawn Robot**: Places the TurtleBot3 at position (0.0, 0.0, 0.01) in the world
5. **RVIZ2**: Opens visualization with my pre-configured display settings

### World Files

I have multiple world files for different purposes:

- **final.sdf** - The main world I use for navigation (currently active in launch file)
- **my_world.sdf** - Alternative environment layout
- **test_world.sdf** - Simplified world for testing features

To switch worlds, just modify the `world_file` path in the launch file.

## Dependencies

Required packages (specified in `package.xml`):

- **turtlebot3_gazebo** - TurtleBot3 models and simulation setup
- **launch_ros** - ROS 2 launch system
- **launch** - Generic launch functionality
- **gazebo_ros** - Gazebo-ROS 2 integration
- **rviz2** - 3D visualization tool

## Building

Build the package from your workspace:

```bash
cd ~/myeq_ws
colcon build --packages-select tb_sim
source install/setup.bash
```

## Usage

### Set TurtleBot3 Model (Required)

Before launching, set the robot model:

```bash
export TURTLEBOT3_MODEL=burger
```

Or add it to your `~/.bashrc` for persistence:

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### Launch the Simulation

Start everything with one command:

```bash
ros2 launch tb_sim start_world_and_robot.launch.py
```

This will:
- Open Gazebo with the simulation world
- Spawn the TurtleBot3 robot at the origin
- Start RVIZ2 with my custom configuration
- Enable `use_sim_time` for all nodes

### What You'll See

**Gazebo Window:**
- The simulation world loaded from `final.sdf`
- TurtleBot3 robot at the starting position
- Any obstacles or structures defined in the world

**RVIZ2 Window:**
- Robot model visualization
- Sensor data (laser scans, camera feeds)
- TF frames
- Map and navigation visualizations (if Nav2 is running)

## Configuration

### Changing the World

To use a different world file, edit `start_world_and_robot.launch.py`:

```python
world_file = os.path.join(tb_sim_dir, 'worlds', 'my_world.sdf')  # Change here
```

### Changing Spawn Position

Modify the spawn arguments in the launch file:

```python
arguments=['-entity', TURTLEBOT3_MODEL, '-file', urdf_file, 
           '-x', '1.0',    # Change X position
           '-y', '2.0',    # Change Y position
           '-z', '0.01'],  # Usually keep Z at 0.01
```

### RVIZ Configuration

To update the RVIZ configuration:
1. Launch the simulation
2. Adjust RVIZ displays as needed
3. Save configuration: File → Save Config As
4. Overwrite `rviz/myeq_rviz_config.rviz`

## Troubleshooting

### Gazebo Doesn't Start
- Ensure Gazebo is installed: `gazebo --version`
- Check if the world file exists in the `worlds/` directory
- Verify `TURTLEBOT3_MODEL` environment variable is set

### Robot Doesn't Spawn
- Confirm `turtlebot3_gazebo` package is installed
- Check terminal output for spawn errors
- Verify the model file path is correct

### RVIZ Crashes or Shows Errors
- The RVIZ config might reference topics that don't exist yet
- Try launching without RVIZ first to verify simulation works
- You can comment out the `rviz_node` in the launch file temporarily

### "use_sim_time" Warnings
- This is normal - the launch file sets `use_sim_time: true`
- All nodes should use Gazebo's simulated time instead of system time

## Integration with Other Packages

This package is designed to work with:

- **tb_nav** - Navigation stack (SLAM, Nav2, localization)
- **waypoint_nav_pkg** - Waypoint-based navigation control

Typical workflow:
1. Launch this simulation package first
2. Launch navigation stack (`tb_nav`)
3. Launch waypoint navigation (`waypoint_nav_pkg`)

## Notes

- The robot spawns at coordinates (0.0, 0.0, 0.01) by default
- All simulation runs with `use_sim_time: true` enabled
- The package uses the TurtleBot3 'burger' model by default
- World files are in SDF (Simulation Description Format)
- RVIZ config is pre-tuned for my robot's sensors and navigation

## Future Improvements

- Add launch arguments to select world files without editing code
- Include different robot models (waffle, waffle_pi)
- Add optional sensor noise parameters
- Create more complex world environments
- Add launch argument for spawn position

---

**Last Updated**: October 31, 2025
