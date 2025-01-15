# PDE4430 - Mobile Robotics - Coursework 2

## Project Description

This repository contains the implementation of a differential drive robot that can move spheres in a given environment. The robot uses a lidar sensor to detect and visualize obstacles.


## Prerequisites

- Ubuntu (recommended: 20.04)
- ROS Noetic
- Gazebo

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/AiswaryaShajan/cw2.git
    ```

2. Navigate to the catkin workspace:
    ```bash
    cd ~/catkin_ws
    ```

3. Build the workspace:
    ```bash
    catkin_make
    ```

4. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

## Running the Simulation

1. Launch the Gazebo environment with the robot:
    ```bash
    roslaunch cw_2 robot.launch
    ```

2. View the robot in RViz:
    ```bash
    roslaunch cw_2 rviz.launch
    ```

## Known Issues

- All the necessary robot transforms may not be created at the moment, only the ones necessary for visualization.
- For some cases, the spheres move when the robot is in close proximity. Need to work on the collision properties of the robot.

## Understanding The ‘Sphere Pusher’ – URDF

![Robot Image](cw_2/images/rviz.jpg)  


- **Differential Drive**: The robot uses a differential drive mechanism controlled by a `differential_drive_controller` plugin in Gazebo.
  - Left Wheel: `left_wheel_joint`
  - Right Wheel: `right_wheel_joint`
  - Wheel Separation: 0.3 meters
  - Wheel Diameter: 0.2 meters
  - Torque: 5.0 Nm
  - Odometry: Published on the `/odom` topic

- **Base Link**: The main body of the robot (`base_link`) has a box shape with dimensions 0.5 x 0.3 x 0.07 meters. The base link is connected to a dummy link (`dummy_link`) via a fixed joint.
- **Wheels**: The robot has two wheels (`left_wheel` and `right_wheel`), each with a radius of 0.1 meters and a length of 0.04 meters. The wheels are attached to the base link via continuous joints.
- **LiDAR Sensor**: The robot is equipped with a lidar sensor (`lidar_link`) mounted on the base link. The lidar sensor is used to detect obstacles and visualize the environment.
  - Sensor Type: Ray
  - Horizontal Scan: 360 samples with a resolution of 1
  - Range: 0.2 to 12 meters
  - Noise: Gaussian with a mean of 0.0 and standard deviation of 0.01
  - Output: Published on the `/scan` topic as `sensor_msgs/LaserScan`

## Visualizing in RViz

In this project, RViz is used to visualize the differential drive robot and its environment, including the obstacles detected by the lidar sensor.

### Launching RViz

The `rviz.launch` file in this repository sets up the necessary nodes and parameters to launch RViz with the robot's URDF model and sensor data:

- **Robot Description**: The URDF model of the robot is loaded using the `robot_description` parameter.
- **Joint State and Robot State Publishers**: These nodes publish the joint states and the robot's state to ROS topics, allowing RViz to visualize the robot's configuration.
- **Static Transform Publisher**: This node publishes a static transform between the lidar sensor frame and the world frame and that with `base_link`, ensuring that the lidar data is correctly positioned in the visualization.

### Setting Up RViz

1. **Add Robot Model**:
    - In the RViz display panel, click "Add".
    - Select "RobotModel" and click "OK".
    - Set the "Robot Description" parameter to `robot_description`.
    - Set the "Fixed Frame" to `base_link` to see the robot model.

2. **Visualize Odometry**:
    - In the RViz display panel, click "Add".
    - Select "Odometry" and click "OK".
    - Set the "Topic" parameter to `/odom`.
    - Set the "Fixed Frame" to `odom` to see the robot's teleoperation.

3. **Add LaserScan**:
    - In the RViz display panel, click "Add".
    - Select "LaserScan" and click "OK".
    - Set the "Topic" parameter to `/scan`.
    - Set the "Fixed Frame" to `world` to visualize the laser lines.

    ![laser](cw_2/images/rviz_laser_scan.JPG) 


Once RViz is running, you can observe the robot's movements and the lidar sensor's data in real-time.

### Launching Gazebo

The `gazebo.launch` file in this repository sets up the necessary environment variables, parameters, and nodes to launch Gazebo with the robot's URDF model and the specified world:

- **GAZEBO_MODEL_PATH**: The environment variable is set to include the directory containing the robot models.
- **Robot Description**: The URDF model of the robot is loaded using the `robot_description` parameter.
- **World and Sim Time**: The world file and simulation time are set using arguments.
- **Gazebo Server and GUI**: Nodes to start the Gazebo server (`gzserver`) and GUI client (`gzclient`) are included.
- **Spawn Robot**: The robot is spawned in the Gazebo environment using the `spawn_model` node with the specified position and orientation.

![Gazebo](cw_2/images/gazebo.jpg)


## Using teleop_twist_keyboard

`teleop_twist_keyboard` is a ROS package that allows you to control a robot using keyboard inputs. It publishes velocity commands to the `/cmd_vel` topic, which can be used to drive the differential drive robot.

1. Install the `teleop_twist_keyboard` package if you haven't already:
    ```bash
    sudo apt-get install ros-<your-ros-distro>-teleop-twist-keyboard
    ```

2. Launch the `teleop_twist_keyboard` node:
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

3. Use the keyboard to control the robot. The keys are mapped as follows:
    - `i`: Move forward
    - `k`: Stop
    - `,`: Move backward
    - `j`: Turn left
    - `l`: Turn right
    - `u`: Forward-left
    - `o`: Forward-right
    - `m`: Backward-left
    - `.`: Backward-right
