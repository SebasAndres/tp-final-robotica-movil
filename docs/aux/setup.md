# Setup

## Prerequisites

- Docker installed on your system
- X11/Wayland display server running (for CoppeliaSim GUI)

## Building the Docker Image

From the repository root:

```bash
cd docker_ros
./start-docker.sh build
```

This builds an image (`ros2_robotica`) that includes:
- ROS2 Humble
- CoppeliaSim 4.7.0 rev2
- simROS2 plugin compiled with all required message types
- `robmovil_msgs` custom messages
- VS Code

### Registered ROS2 Message Types

The following message types are registered in the simROS2 plugin (`meta/interfaces.txt`) during the Docker build:

| Package | Messages |
|---|---|
| `geometry_msgs` | PoseArray, PoseStamped, PoseWithCovarianceStamped, TransformStamped |
| `sensor_msgs` | LaserScan, Imu, Joy |
| `nav_msgs` | Odometry, Path |
| `std_msgs` | Float64, Int32 |
| `trajectory_msgs` | JointTrajectoryPoint, JointTrajectory |
| `rosgraph_msgs` | Clock |
| `robmovil_msgs` | MultiEncoderTicks, EncoderTicks, Trajectory, TrajectoryPoint, LandmarkArray, Landmark |

If CoppeliaSim reports an "Unsupported message type" error, add the missing type to the echo block in `docker_ros/Dockerfile` and rebuild the image.

## Starting the Container

```bash
cd docker_ros
./start-docker.sh start
```

## Opening a Terminal Inside the Container

```bash
./start-docker.sh open
```

## Stopping / Removing the Container

```bash
./start-docker.sh stop      # stop
./start-docker.sh down       # stop and remove
```

## Building the ROS2 Workspace (Inside the Container)

After opening a terminal inside the container:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash 
colcon build --packages-select robmovil_msgs 
colcon build --packages-select modelo_omnidireccional 
source install/setup.bash 
```

The workspace sources are mounted from the host at:
- `talleres/` -> `/root/ros2_ws/src/robotica`
- `tp_final/modelo_omnidireccional/` -> `/root/ros2_ws/src/tp_final`
- `tp_final/coppeliaSim/` -> `/root/ros2_ws/src/tp_final/coppeliaSim`
