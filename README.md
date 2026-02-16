# ROS2_V4 (CR10 + RG6) Installation and Configuration Guide

---

### **Introduction**
`DOBOT_6Axis_ROS2_V4_CR10_RG6` is a customized ROS2 Software Development Kit (SDK) based on Dobot’s TCP/IP communication framework.  
It is developed with ROS2/C++ and Python, following the Dobot TCP/IP protocol to establish Socket-based TCP connections with the robot controller and provide user-friendly APIs for development and control.

This customized version is focused on:
- Keeping **CR10 only** (all other robot models removed)
- Integrating **RG6 gripper description** into the robot model
- Supporting simulation and visualization workflows with RViz/Gazebo
- Preparing RG6 actuation/controller integration (currently under development)

---

### **Prerequisites**

1. **Network Configuration**
   - **Wired Connection**: Controller IP is `192.168.5.1`. Set the computer to a fixed IP in the same subnet.
   - **Wireless Connection**: Controller IP is `192.168.1.6`.
   - Use `ping` to confirm communication with the controller.

2. **System Requirements**
   - **Operating System**: Ubuntu 22.04
   - **ROS Version**: ROS2 Humble

---

### **Installation and Configuration Steps**

#### **1. Source Code Compilation**
1. Download the source code:
   ```bash
   mkdir -p ~/dobot_ws/src
   cd ~/dobot_ws/src
   git clone <YOUR_NEW_REPOSITORY_URL>.git
   cd ~/dobot_ws
Compile the source code:

colcon build
source install/local_setup.sh
Set environment variables:

echo "source ~/dobot_ws/install/local_setup.sh" >> ~/.bashrc
Configure robotic arm connection IP (default wired):

echo "export IP_address=192.168.5.1" >> ~/.bashrc
Specify robotic arm model (CR10 only in this customized package):

echo "export DOBOT_TYPE=cr10" >> ~/.bashrc
Apply configuration:

source ~/.bashrc
To modify configuration later, edit ~/.bashrc.

Feature Demonstrations
1. Simulation Environment Usage
RViz Model Loading:

ros2 launch dobot_rviz dobot_rviz.launch.py
Loads CR10 + RG6 model for visualization.
![rviz](/image/rviz.jpg)

MoveIt Virtual Demo:

ros2 launch dobot_moveit moveit_demo.launch.py
Adjust joint values and click Plan and Execute to visualize motion.
![moveit](/image/moveit.jpg)

Gazebo Simulation:

ros2 launch dobot_gazebo dobot_gazebo.launch.py
Launches Gazebo simulation with CR10 + RG6 model.
![gazebo](/image/gazebo.jpg)

Gazebo and MoveIt Integration:

Launch Gazebo and MoveIt:

ros2 launch dobot_gazebo gazebo_moveit.launch.py
ros2 launch dobot_moveit moveit_gazebo.launch.py
Plan and execute from MoveIt; motion synchronizes with Gazebo.
![service](/image/node.jpg)

2. Controlling a Real Robotic Arm (CR10)
Connect to the robotic arm:

ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
View service list:

ros2 service list
![service](/image/service.jpg)

Control robotic arm with MoveIt:

ros2 launch dobot_moveit dobot_moveit.launch.py
![service](/image/dobot_moveit.jpg)

Note: Ensure robot is in remote TCP mode and enabled.

Enable the robotic arm:

Use rqt service caller to call:

/dobot_bringup_ros2/srv/EnableRobot
![gazebo](/image/rqt.jpg)

Key topics:

/joint_states_robot: Joint angle/state feedback

/dobot_msgs_v4/msg/ToolVectorActual: Cartesian pose feedback
![gazebo](/image/topic.jpg)

RG6 Gripper Status in This Package
✅ RG6 integrated in robot description and loaded with CR10 model

✅ RG6 visualized correctly in RViz

✅ RG6 appears correctly in Gazebo model

⚠️ RG6 control/controller interface is currently WIP

⚠️ Open/close command pipeline will be added in upcoming updates

Essential Toolkit Installation
1. Gazebo Installation
Installation command:

sudo apt install ros-humble-gazebo-*
Environment variable configuration:

echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
source ~/.bashrc
Test run:

ros2 launch gazebo_ros gazebo.launch.py
2. MoveIt Installation
Installation command:

sudo apt-get install ros-humble-moveit
Credits
This customized package is built on top of and/or references the following open-source projects:

Dobot Official ROS2 SDK (V4)

Repository: https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4

Contribution: Base ROS2 TCP/IP SDK, robot bringup, MoveIt/Gazebo/RViz workflows

RG6 Gripper Description

Repository: https://github.com/Zhengxuez/rg6_gripper_description

Contribution: RG6 gripper model/description used for integration with CR10

ROS2 Humble Ecosystem

gazebo_ros, ros2_control, moveit and standard ROS2 tools used in this project workflow

Please keep upstream attribution and license notices when redistributing or modifying this package.

Notes
This customized repository supports CR10 only.

Ensure correct IP/network settings before attempting hardware control.

When operating real hardware, strictly follow safety procedures.

Validate trajectories in simulation before real execution.

RG6 controller support is under active development in this branch.

Document Version: CR10+RG6 V1
Last Updated: February 16, 2026
