#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2节点示例-发布“Hello RoboTUM 2025! OOP”日志信息, 使用面向对象的实现方式
@Author: Pedro
@Description: ROS2 Node Example - Publishes "Hello RoboTUM 2025! OOP" log messages using an object-oriented programming approach
"""

import rclpy                                     # ROS2 Python接口库
# ROS2 Python client library

from rclpy.node import Node                      # ROS2 节点类
# ROS2 Node class

import time
# 导入time模块，用于控制循环时间
# Import time module for loop timing control

"""
创建一个HelloRoboTUM节点, 初始化时输出“hello RoboTUM”日志
Create a HelloRoboTUM node that logs "Hello RoboTUM" messages upon initialization
"""
class HelloRoboTUMNode(Node):                                     # 继承父类 Node，来自 ROS2
    # Inherits from the Node base class provided by ROS2
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        # Initialize the parent class Node

        while rclpy.ok():                                         # ROS2系统是否正常运行
            # Check if ROS2 is still running

            self.get_logger().info("Hello RoboTUM 2025! OOP")     # ROS2日志输出
            # Output a log message via the ROS2 logger

            time.sleep(0.5)                                       # 休眠控制循环时间
            # Sleep for 0.5 seconds to control message frequency

def main(args=None):                                              # ROS2节点主入口main函数
    # Main function - Entry point for the ROS2 node

    rclpy.init(args=args)                                         # ROS2 Python接口初始化
    # Initialize the ROS2 Python client library

    node = HelloRoboTUMNode("node_hellorobotum_class")            # 这里是自己创建的类，创建ROS2节点对象并进行初始化
    # Create and initialize a ROS2 node object from the custom class

    node.destroy_node()                                           # 销毁节点对象
    # Destroy the node object

    rclpy.shutdown()                                              # 关闭ROS2 Python接口
    # Shutdown the ROS2 Python client library

