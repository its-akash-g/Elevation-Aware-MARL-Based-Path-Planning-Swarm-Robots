# Elevation-Aware Multi-Agent 3D Path-Planning Swarm Robots

This repository contains the simulation framework and research codebase for evaluating an **elevation-aware** multi-agent path-planning system using distributed consensus in ROS 2 Humble and Gazebo.

## System Architecture & Features
* **Environment**: Custom 3D rough-terrain Gazebo world.
* **Kinematics**: Managed via `ros2_control` and differential drive controllers.
* **Metrics**: Real-time energy consumption tracking ($C_i$) based on elevation gain and distance.
* **Coordination**: Distributed consensus for leader election and navigation synchronization.

---

## Prerequisites
* **OS**: Ubuntu 22.04 / Fedora (via Docker or native)
* **ROS 2 Distribution**: Humble Hawksbill
* **Simulator**: Gazebo Classic / Ignition (depending on your setup)

---

## Installation & Setup

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/its-akash-g/Elevation-Aware-MARL-Based-Path-Planning-Swarm-Robots.git](https://github.com/its-akash-g/Elevation-Aware-MARL-Based-Path-Planning-Swarm-Robots.git) ~/Projects/pp1
