# Vortex Utility Nodes

A collection of lightweight ROS2 utility nodes for testing, debugging, and operating the Vortex AUV.

These are standalone helper nodes — not core pipeline components. Use them to inspect data, convert formats, or bridge gaps during development and field operations.

## Nodes

| Node | Description |
|------|-------------|
| `euler_odometry_publisher_node` | Subscribes to `odometry` (`nav_msgs/Odometry`) and republishes on `odometry/euler` with orientation converted from quaternions to Euler angles (roll, pitch, yaw). |

## Build
```bash
colcon build --packages-select vortex_utility_nodes
source install/setup.bash
```

## Run
```bash
ros2 run vortex_utility_nodes euler_odometry_publisher_node
```

## Adding New Nodes

1. Create your node source file in `src/`
2. Add an `add_executable` entry in `CMakeLists.txt`
3. Add the install target to `lib/${PROJECT_NAME}`
4. Update this table

## Current Dependencies

- `rclcpp`
- `nav_msgs`
- `vortex_utils`
- `Eigen3`
