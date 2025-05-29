# RoboTUM ROS2 Workshop 🚀

欢迎参加由 **RoboTUM** 组织的 ROS 2 Workshop！  
本项目旨在帮助你快速上手 ROS 2，并掌握其核心概念与编程实践。  
我们将通过一系列结构清晰的示例，带你逐步掌握 ROS 2 中的 Topic、Service、Action、Node、Launch 文件等内容。

---

## 📚 Workshop 目标

- 理解 ROS 2 的基本通信机制（Topic / Service / Action）
- 学会编写和运行简单的 ROS 2 节点（以 Python 为主，同时提供一部分 C++ 示例）
- 掌握自定义 Interface 的创建与使用
- 能够使用 launch 文件组织项目结构
- 掌握如何构建和运行一个完整的 ROS 2 项目

---

## 🛠️ 环境要求

推荐开发环境如下：

- **ROS 2 Rolling**（适配最新特性）  
- **Ubuntu 24.04**
- **colcon**
- **Python 3.10+**
- **C++17**

📝 *说明*：本项目理论上也兼容 **ROS 2 Humble + Ubuntu 22.04** 组合。

---

## 📄 Workshop 讲义

👉 [点击查看 Notion 上的讲义（适合初学者）](https://www.notion.so/starryocean/ROS2-Workshop-ROS2-ROS2-Workshop-Handouts-for-ROS2-beginners-1ff866ba436e8056b00fc457636b7952?source=copy_link)

---

## 🚀 快速开始

```bash
# 1. 克隆本仓库
git clone https://github.com/Perdo33Messi/RoboTUM_ROS2_Workshop.git
cd RoboTUM_ROS2_Workshop

# 2. 创建 ROS 2 工作区并构建
mkdir -p ~/ros2_ws/src
cp -r . ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/rolling/setup.bash  # 或 source /opt/ros/humble/setup.bash
colcon build



---

## 🎯 面向人群

- 对 ROS / 机器人感兴趣的本科生或研究生  
- 没有 ROS 2 编程经验的初学者  
- 想从原理层面理解 ROS 通信模型的开发者

---

## ❤️ 致谢

本 Workshop 由 TUM RoboTUM 成员整理与分享，欢迎大家通过 Issue 或 Pull Request 提出建议与改进。  
愿你在机器人开发的路上，越走越稳，越飞越高！✨
