

# Jo Sim
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-34C759?style=flat-square&logo=ros)](https://docs.ros.org/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-FF6B35?style=flat-square&logo=gazebo)](https://gazebosim.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?style=flat-square&logo=docker)](https://www.docker.com/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?style=flat-square&logo=ubuntu)](https://ubuntu.com/)
[![Nav2](https://img.shields.io/badge/Nav2-Latest-4285F4?style=flat-square&logo=ros)](https://navigation.ros.org/)

A comprehensive ROS 2 robot description package for the Jo tracked robot platform, featuring complete robot modeling, simulation, and navigation capabilities using Gazebo (Harmonic) and ROS 2 Jazzy.

<div align="center">
  <img src="res/description.png" alt="Jo Overview" height="400"/>
</div>


## Key Features

- **Complete URDF/Xacro Model**
- **Gazebo Harmonic Simulation**
- **Sensor Suite**:
  - Velodyne LiDAR (3D point cloud)
  - Front and back depth cameras (RGB-D)
  - IMU sensor for orientation/acceleration
  - Track simulation
- **Navigation Ready**: Pre-configured Nav2 stack for autonomous (mapless) navigation
- **Dynamic Obstacle Detection**: Real-time 3D tracking of pedestrians, vehicles, and other dynamic objects via sensor fusion (depth + LiDAR + YOLOv11)
- **Docker Support**: Full containerized environment with GPU acceleration
- **RViz Visualization**: Pre-configured visualization configs for display, simulation, and navigation


## Packages
- `jo_description` — URDF model of the robot, complete with sensors
- `jo_sim` — Simulation package; used to launch Gazebo sim and GLIM SLAM
- `jo_navigation` — Configs and launch files for localization (EKF) and navigation (Nav2)
- `onboard_detector` — Real-time 3D dynamic obstacle detection and tracking (depth + LiDAR + YOLO fusion)



# Quick Start

## Prerequisites
To correctly install the docker with GPU access, these steps need to be followed:
1. Install Docker Engine with their [guide](https://docs.docker.com/engine/install/ubuntu/)
2. Allow Docker usage as non-root user ([guide](https://docs.docker.com/engine/install/linux-postinstall/))
3. Correctly install [nVidia drivers](https://github.com/oddmario/NVIDIA-Ubuntu-Driver-Guide)
4. Install the [`nvidia-container-toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Building and Running

### 1. **Start and build the docker image**:
```bash
docker compose up --build
```

### 2. **Access the container**:
```bash
docker compose exec jo-sim bash
```

### Inside the Container

Once in the container, you can run various launch files:

### 1. Run Full Simulation with Gazebo
```bash
ros2 launch jo_sim launch_sim.launch.py rviz:=true
```

This launches:
- Gazebo simulator with the robot in an office world
- Robot state publisher 
- ROS-Gazebo bridge for sensor/actuator communication
- RViz visualization

Adding ```glim:=true``` to the launch will run GLIM configured to work with the simulation.

The simulation also includes:
- Simulated tracks using Gazebo's TrackedVehicle and TrackController plugins
- Simulated IMU
- Simulated 3D GPU lidar using RGLGazeboPlugin
- Two simulated RGBD cameras
- Several premade worlds, most coming from [this repo](https://github.com/leonhartyao/gazebo_models_worlds_collection.git)

To control the robot you will need to open another terminal and run 
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```
<div align="center">
  <img src="res/rviz.png" alt="Rviz view" height="250"/>
  <img src="res/gazebo.png" alt="Gazebo View" height="250"/>
</div>

### 2. Localization stack
In this implementation, GLIM will not publish on ```/tf```. This means that the localization module is needed to have a non inertial visualization.

#### Local Odometry
Launching 
```bash
ros2 launch jo_navigation localization.launch.py use_sim_time:=true
```
will run an instance of the EKF node from [```robot_localization```](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html) configured to estimate ```odom``` to ```base_link```, allowing to use the first as fixed frame in RViz.

#### Global Odometry
To achieve global localization, launch 
```bash
ros2 launch jo_navigation localization_gps.launch.py use_sim_time:=true
```
This will launch two instances of the EKF node. One performs local odometry (in a manner identical to the previous launch file), while the other uses GPS data to perform global localization, publishing the transform from ```map``` to ```odom```.

**Note:** As of now this localization is still not functional in the simulation, as there is not yet a working implementation o simulated GPS.

### 3. Autonomous Navigation with Nav2
Similarly to the odometry, there are two main modalities for the navigation.

#### Local navigation
To perform local navigation, launch
```bash
ros2 launch jo_navigation navigation_local.launch.py use_sim_time:=true rviz:=true
```
This will require the **Local Odometry** to be up and running.
This will launch the navigation stack and RViz configured to see the perception stack, the local costmap as well as the planned path. To give the robot a desired waypoint you need to click on RViz 2D Goal Pose and select the desired point on the map.
Given that it's a local navigation, the fixed frame needs to be set to ```odom```.


#### Global Navigation
To perform global navigation, launch
```bash
ros2 launch jo_navigation navigation_gps.launch.py use_sim_time:=true rviz:=true
```
This will require the **Global Odometry** to be up and running. This is a modified version of the local navigation stack able to correctly use the ```map``` frame published and the new odometry sources.


<div align="center">
  <img src="res/navigation.gif" alt="Navigation Stack" height="530"/>
</div>


## ROS Topics

### Sensor Topics (Published by Gazebo)
```
/odom                           # TrackedVehicle Odometry (not working properly)
/imu/data                       # IMU data
/velodyne_points                # Lidar point cloud
/front_camera/image             # Front RGB camera image
/front_camera/camera_info       # Front camera intrinsics
/front_camera/points            # Front camera point cloud
/front_camera/depth             # Front depth image
/back_camera/image              # Back RGB camera image
/back_camera/camera_info        # Back camera intrinsics
/back_camera/points             # Back camera point cloud
/back_camera/depth              # Back depth image
```

### Control Topics (Subscribed)
```
/cmd_vel                        # Velocity command (geometry_msgs/Twist)
```



### Custom Worlds
Place custom world files in `jo_description/worlds/external/worlds/`:
```bash
ros2 launch jo_sim launch_sim.launch.py world:=/path/to/custom.world
```



## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [URDF Documentation](http://wiki.ros.org/urdf)
- [ROS 2 Control Documentation](https://control.ros.org/)

