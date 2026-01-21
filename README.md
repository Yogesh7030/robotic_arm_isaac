# Robotic Arm – ROS 2, MoveIt 2, and Isaac Sim

This repository contains a **ROS 2 workspace for a robotic arm**, integrated with **MoveIt 2** for motion planning and **NVIDIA Isaac Sim** for simulation.  
The project is distributed using **Docker** to ensure reproducibility and ease of use.

---

git clone <repository-url>
cd robotic_arm_isaac
docker compose up -d
docker exec -it moveit2 bash
ros2 launch my_robot_bringup display.launch.xml

---

## Workspace Overview

The workspace consists of **five ROS 2 packages**:

### 1. `my_robot_description`
- Robot URDF/XACRO description
- Includes a **USD file for Isaac Sim** containing:
  - Action Graph for ROS–Isaac communication
  - Joint state publishers
  - Joint command subscribers
- Serves as the primary robot model for both MoveIt and Isaac Sim

---

### 2. `my_robot_moveit_config`
- MoveIt 2 configuration package
- Planning groups, controllers, and planners
- OMPL-based motion planning setup

---

### 3. `my_robot_bringup`
- Launch files for the complete system
- Starts:
  - Robot State Publisher
  - MoveIt `move_group`
  - RViz visualization

---

### 4. `my_robot_interfaces`
- Custom ROS 2 interfaces
- Defines messages and services used for robot control

---

### 5. `my_robot_commander`
- High-level MoveIt commander package
- Allows commanding the robot using:
  - Joint-space goals
  - End-effector pose goals

---

## Docker Setup

The project uses **prebuilt Docker images**.  
Users do **not** need to build the image locally.

Two Docker Compose configurations are provided:
- **ROS 2 + MoveIt 2 only**
- **ROS 2 + MoveIt 2 with Isaac Sim communication**

---

## Running the System

### 1. ROS 2 + MoveIt 2 (without Isaac Sim)

```bash
docker compose up -d
docker exec -it moveit2 bash

## Inside the container:
```bash
ros2 launch my_robot_bringup display.launch.xml

### 1. ROS 2 + MoveIt 2 (with Isaac Sim)
## This mode assumes Isaac Sim is running on the host system.

```bash
docker compose up -d
docker exec -it moveit2 bash

## Inside the container:
```bash
ros2 launch my_robot_bringup display.launch.xml

Isaac Sim Integration Notes
---------------------------

- Isaac Sim runs directly on the **host machine**
- ROS 2 communication is handled via the **Action Graph defined in the USD file**
- The USD file:
  - Publishes joint states
  - Subscribes to joint command topics

* * *

RViz Visualization
------------------

To visualize the robot model and TF frames in RViz:

1. Add a **RobotModel** display
2. Set the **Description Topic** to:

## /robot_description
## This ensures RViz correctly displays the robot model when running inside Docker.

Motion Planning
---------------

- Planning speed can be adjusted directly within the MoveIt planner

- Supported planning modes:
  - Joint-space motion planning
  - End-effector pose planning

* * *

License
-------

This project is licensed under the **MIT License**.




