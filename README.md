# RoboTUM ROS2 Workshop 🚀

Welcome to the **RoboTUM** ROS 2 Workshop!  
This project is designed to help you get started with ROS 2 and understand its core concepts through hands-on programming.  
We'll walk you through key components like Topics, Services, Actions, Nodes, and Launch files with clear and structured examples.

---

## 📚 Workshop Goals

- Understand the basic communication mechanisms in ROS 2 (Topic / Service / Action)
- Learn how to write and run simple ROS 2 nodes (mainly in Python, with some C++ examples)
- Create and use custom interfaces
- Use launch files to organize project structure
- Build and run a complete ROS 2 project from scratch

---

## 🛠️ Environment Requirements

Recommended development environment:

- **ROS 2 Rolling** (latest features)  
- **Ubuntu 24.04**
- **colcon**
- **Python 3.10+**
- **C++17**

📝 *Note*: This project is also expected to be compatible with **ROS 2 Humble + Ubuntu 22.04**.

---

## 📄 Workshop Handouts

👉 [Click here to view the Notion handouts (beginner-friendly)](https://www.notion.so/starryocean/ROS2-Workshop-ROS2-ROS2-Workshop-Handouts-for-ROS2-beginners-1ff866ba436e8056b00fc457636b7952?source=copy_link)

---

## 🚀 Quick Start

```bash
# 1. Clone this repository
git clone https://github.com/Perdo33Messi/RoboTUM_ROS2_Workshop.git
cd RoboTUM_ROS2_Workshop

# 2. Create a ROS 2 workspace and build
mkdir -p ~/ros2_ws/src
cp -r . ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/rolling/setup.bash  # or source /opt/ros/humble/setup.bash
colcon build

---

## 🎯 Target Audience

- Undergraduate or graduate students interested in robotics or ROS  
- Beginners with little or no experience in ROS 2  
- Developers who want to understand the principles behind ROS communication models

---

## ❤️ Acknowledgment

This workshop is organized and maintained by members of **TUM RoboTUM**.  
Feel free to submit suggestions or improvements via **Issues** or **Pull Requests**.  
We wish you a steady and soaring journey in the world of robotics development! ✨
