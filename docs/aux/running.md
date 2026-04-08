# Running the System

All commands below are run **inside the Docker container** (`./start-docker.sh open`).

## 1. Start CoppeliaSim

```bash
coppeliaSim /root/ros2_ws/src/tp_final/coppeliaSim/omni_ekf.ttt
```

Press the play button in CoppeliaSim to start the simulation. This publishes:
- `/robot/encoders` (robmovil_msgs/MultiEncoderTicks)
- `/robot/front_laser/scan` (sensor_msgs/LaserScan)
- `/posts` (geometry_msgs/PoseArray)
- `/tf` transforms: `map -> odom`, `odom -> base_link_gt`, `base_link -> front_laser`

And subscribes to:
- `/robot/front_left_wheel/cmd_vel` (std_msgs/Float64)
- `/robot/front_right_wheel/cmd_vel` (std_msgs/Float64)
- `/robot/rear_left_wheel/cmd_vel` (std_msgs/Float64)
- `/robot/rear_right_wheel/cmd_vel` (std_msgs/Float64)

## 2. Run the Odometry Node (Section 1)

In a second terminal:

```bash
ros2 run modelo_omnidireccional omni_odometry_node
```

This node:
- Subscribes to `/robot/cmd_vel` (Twist) and converts to individual wheel velocities (inverse kinematics)
- Subscribes to `/robot/encoders` and computes odometry (forward kinematics)
- Publishes `/robot/odometry` (nav_msgs/Odometry) and the `odom -> base_link` TF transform

## 3. Run the Trajectory Follower (Section 2)

In a third terminal:

```bash
ros2 run modelo_omnidireccional trajectory_follower_node
```

With custom gains:

```bash
ros2 run modelo_omnidireccional trajectory_follower_node --ros-args \
  -p kp_x:=1.0 \
  -p kp_y:=1.0 \
  -p kp_theta:=1.5 \
  -p position_tolerance:=0.1 \
  -p angle_tolerance:=0.15 \
  -p control_rate:=20.0
```

This node:
- Reads the `map -> base_link` TF as pose feedback
- Computes P control errors in the robot frame
- Publishes Twist on `/robot/cmd_vel`
- Follows a 2m square trajectory with outward-facing orientation

## Useful Debugging Commands

```bash
# List active topics
ros2 topic list

# Monitor odometry
ros2 topic echo /robot/odometry

# Monitor velocity commands
ros2 topic echo /robot/cmd_vel

# Check TF tree
ros2 run tf2_tools view_frames

# Manual velocity command (for testing)
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
