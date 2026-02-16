# 🤖 ROS2_V4 (CR10 + RG6) — Installation & Configuration Guide

> **Package:** `DOBOT_6Axis_ROS2_V4_CR10_RG6`  
> **Document Version:** CR10+RG6 V1  
> **Last Updated:** **February 16, 2026**

---

## 📌 Introduction

`DOBOT_6Axis_ROS2_V4_CR10_RG6` is a customized ROS2 Software Development Kit (SDK) based on **Dobot’s TCP/IP communication framework**.

It is developed with **ROS2 / C++** and **Python**, following the Dobot TCP/IP protocol to establish **Socket-based TCP connections** with the robot controller and provide user-friendly APIs for development and control.

### ✅ What’s Customized in This Version

This customized fork is focused on:

- ✅ Keeping **CR10 only** (all other robot models removed)
- ✅ Integrating the **RG6 gripper description** into the CR10 robot model
- ✅ Supporting simulation + visualization workflows (**RViz / Gazebo / MoveIt**)
- ⚠️ Preparing RG6 actuation/controller integration (**WIP**)

---

## ✅ Prerequisites

### 1) 🌐 Network Configuration

Choose the correct controller IP depending on your connection:

- **Wired Connection:** `192.168.5.1`  
  Set your PC to a fixed IP in the same subnet (example: `192.168.5.100`)

- **Wireless Connection:** `192.168.1.6`

Verify connectivity:

\```bash
ping 192.168.5.1
\```

> If ping fails: check subnet settings, firewall rules, and physical connection.

---

### 2) 💻 System Requirements

- **OS:** Ubuntu 22.04  
- **ROS2:** Humble

---

## 🧰 Installation & Configuration

### 1) 📥 Clone the Repository

\```bash
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src
git clone <YOUR_NEW_REPOSITORY_URL>.git
\```

---

### 2) 🏗️ Build the Workspace

\```bash
cd ~/dobot_ws
colcon build
\```

Then source the workspace:

\```bash
source install/local_setup.sh
\```

---

### 3) ⚙️ Configure Environment Variables

Add ROS workspace sourcing and robot configuration to your `~/.bashrc`:

\```bash
echo "source ~/dobot_ws/install/local_setup.sh" >> ~/.bashrc
\```

Set the robot controller IP (**default wired**):

\```bash
echo "export IP_address=192.168.5.1" >> ~/.bashrc
\```

Select robot type (**CR10 only** in this repository):

\```bash
echo "export DOBOT_TYPE=cr10" >> ~/.bashrc
\```

Apply changes:

\```bash
source ~/.bashrc
\```

> To modify later, edit: `~/.bashrc`

---

## 🚀 Feature Demonstrations

## 1) 🧪 Simulation & Visualization

### ✅ RViz Model Loading (CR10 + RG6)

\```bash
ros2 launch dobot_rviz dobot_rviz.launch.py
\```

Loads CR10 + RG6 model for visualization.

![rviz](/image/rviz.jpg)

---

### ✅ MoveIt Virtual Demo (Planning Only)

\```bash
ros2 launch dobot_moveit moveit_demo.launch.py
\```

Adjust joint values in MoveIt and click **Plan** / **Execute** to visualize motion.

![moveit](/image/moveit.jpg)

---

### ✅ Gazebo Simulation

\```bash
ros2 launch dobot_gazebo dobot_gazebo.launch.py
\```

Launches Gazebo simulation with CR10 + RG6 model.

![gazebo](/image/gazebo.jpg)

---

### ✅ Gazebo + MoveIt Integration

Launch Gazebo and MoveIt:

\```bash
ros2 launch dobot_gazebo gazebo_moveit.launch.py
ros2 launch dobot_moveit moveit_gazebo.launch.py
\```

Plan and execute from MoveIt; motion synchronizes with Gazebo.

![node](/image/node.jpg)

---

## 2) 🦾 Controlling a Real CR10 Robot

### ✅ Bringup (Connect to Robot Controller)

\```bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
\```

---

### ✅ View Available Services

\```bash
ros2 service list
\```

![service](/image/service.jpg)

---

### ✅ Control Robot with MoveIt

\```bash
ros2 launch dobot_moveit dobot_moveit.launch.py
\```

![dobot_moveit](/image/dobot_moveit.jpg)

> **Important:** Ensure the robot is in **Remote TCP mode** and **enabled**.

---

### ✅ Enable the Robot (Service Call via rqt)

Use **rqt service caller** to call:

- `/dobot_bringup_ros2/srv/EnableRobot`

![rqt](/image/rqt.jpg)

---

### 📡 Key Topics

- `/joint_states_robot`  
  Joint angle/state feedback

- `/dobot_msgs_v4/msg/ToolVectorActual`  
  Cartesian pose feedback

![topic](/image/topic.jpg)

---

## 🟦 RG6 Gripper Status (Current State)

- ✅ RG6 integrated into robot description and loaded with CR10 model  
- ✅ RG6 visualized correctly in RViz  
- ✅ RG6 appears correctly in Gazebo model  
- ⚠️ RG6 control/controller interface is **Work In Progress**  
- ⚠️ Open/close command pipeline will be added in upcoming updates  

---

## 🧩 Essential Toolkit Installation

## 1) 🏟️ Gazebo Installation

Install:

\```bash
sudo apt install ros-humble-gazebo-*
\```

Configure environment:

\```bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
source ~/.bashrc
\```

Test:

\```bash
ros2 launch gazebo_ros gazebo.launch.py
\```

---

## 2) 🧠 MoveIt Installation

\```bash
sudo apt-get install ros-humble-moveit
\```

---

## 🙏 Credits & Attribution

This customized package is built on top of and/or references:

### ✅ Dobot Official ROS2 SDK (V4)
- Repo: `https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4`
- Contribution: Base ROS2 TCP/IP SDK, bringup, MoveIt/Gazebo/RViz workflows

### ✅ RG6 Gripper Description
- Repo: `https://github.com/Zhengxuez/rg6_gripper_description`
- Contribution: RG6 gripper model/description used for integration with CR10

### ✅ ROS2 Humble Ecosystem
- Tools: `gazebo_ros`, `ros2_control`, `moveit`, standard ROS2 tooling used in this workflow

> Please keep upstream attribution and license notices when redistributing or modifying this package.

---

## ⚠️ Notes & Safety

- This customized repository supports **CR10 only**
- Ensure correct IP/network settings before hardware control
- Follow safety procedures when operating real hardware
- Validate trajectories in simulation before real execution
- RG6 controller support is under active development in this branch

---
