#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2节点示例-发布“Hello RoboTUM 2025!”日志信息, 使用面向过程的实现方式
@Author: Pedro
@Description: ROS2 Node Example - Publishes "Hello RoboTUM 2025!" log messages using a procedural programming approach
"""

import rclpy                                            # ROS2 Python接口库
# ROS2 Python client library

from rclpy.node import Node                             # 引入 ROS2 节点类
# Import the Node class from the ROS2 Python library

import time                                             # 导入time模块，用于控制循环时间
# Import time module for controlling loop timing

def main(args=None):                                    # ROS2节点主入口main函数
    # Main function - Entry point for the ROS2 node

    rclpy.init(args=args)                               # ROS2 Python接口初始化
    # ① Initialize the ROS2 Python client library

    node = Node("node_hellorobotum")                    # 创建ROS2节点对象并进行初始化，“”里面是节点的名字
    # ② Create and initialize a ROS2 node object, the string is the node name

    # ③ Run Main Logic
    while rclpy.ok():                                    # ROS2系统是否正常运行
        # Check if the ROS2 system is running properly

        node.get_logger().info("Hello RoboTUM 2025!")    # ROS2日志输出
        # Output a log message using ROS2 logging system

        time.sleep(0.5)                                  # 休眠控制循环时间
        # Sleep for 0.5 seconds to control loop frequency
    
    node.destroy_node()                                  # 销毁节点对象    
    # Destroy the node object

    rclpy.shutdown()                                     # 关闭ROS2 Python接口
    # ④ Shutdown the ROS2 Python client library
