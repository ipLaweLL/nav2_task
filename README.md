# Differential Drive Robot with Nav2 and Custom Navigation Node

A ROS 2 based diff drive robot featuring custom navigation nodes and optimized parameters for autonomous operation done using Zenoh middleware (NOTE: fastDDS breaks the gazebo environment so please consider using alternate middleware setups, I've simulated mine using Zenoh, CycloneDDS works equally fine)


## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Any middleware implementation that isnt FastDDS
- Gazebo Ignition Fortress
- RViz2
- Python 3.10+
- C++ 14 or newer

### Required Packages
```bash
# Install core dependencies
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
```
# Install Gazebo Ignition-Fortress
sudo apt-get install ignition-fortress


## System Components

### 1. Custom Navigation Node
The specialized navigation implementation includes:
- Enhanced path planning algorithms
- Custom cost functions
- Optimized obstacle avoidance
- Real-time path optimization

### 2. Simulation Environment
Fully integrated Gazebo simulation with:
- Warehouse environment
- Simulated sensors
- Performance monitoring
- Debug visualization

### 3. Localization System
Map-based positioning system using:
- AMCL localization
- Custom map integration
- Real-time position tracking

### 4. Navigation System
Using Nav2 stack and updated parameters

## Running the Complete System

### 1. Start the Zenoh Daemon (if using Zenoh)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### 2. Launch Core System
```bash
# Terminal 1: Launch simulation and robot
ros2 launch nav2_task launch_sim.launch.py

# Terminal 2: Launch rviz
rviz2
Select robotmodel, laserscan, map, plan and marker visualisation plugins

# Terminal 3: Launch localization
ros2 launch nav2_task localization_launch.py map:=./src/nav2_task/maps/warehouse_save.yaml use_sim_time:=true

# Terminal 4: Launch custom navigation
ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true

# Terminal 5: Launch navigation node
ros2 run nav_client nav_client_node

```
## Launch Files Explained

### launch_sim.launch.py
Launches simulation components:
- Gazebo environment
- Robot spawning
- Sensor simulation
- Basic parameters

### localization_launch.py
Initiates localization:
- Map loading
- AMCL setup
- Transform configuration

### navigation_launch.py
Starts navigation stack:
- Custom navigation node
- Parameter loading
- Path planning
- Obstacle avoidance

### nav_client_node.cpp
Interfaces with AMCL node for initialisation, Nav2 nodes to send and publishes marker topics to rviz

## Configuration Files

### nav_params.yaml
Fine tuned to match environmental constraints and robot capabilities

## Development Tools

### Building and Testing
```bash
# Build specific packages inside your development workspace(dev_ws or ros2_ws)
colcon build --packages-select accl_task or colcon build --symlink-install \

## Common Issues and Solutions

### Navigation Issues
1. **Map Not Loading**
   - Verify map path
   - Check file permissions
   - Confirm YAML format

2. **Navigation Failures**
   - Check transform tree
   - Verify localization
   - Review costmap parameters

3. **Simulation Issues**
   - Confirm use_sim_time settings
   - Check Gazebo launch
   - Verify sensor data
