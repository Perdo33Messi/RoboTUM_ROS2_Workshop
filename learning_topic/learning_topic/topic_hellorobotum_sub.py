#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2话题示例-订阅“Hello RoboTUM 2025!”话题消息
@Author: Pedro
@Description: ROS2 Topic Example - Subscribe to the "Hello RoboTUM 2025!" topic message
"""

import rclpy                                     # ROS2 Python接口库
                                                # ROS2 Python client library

from rclpy.node   import Node                    # ROS2 节点类
                                                # ROS2 Node class

from std_msgs.msg import String                  # ROS2标准定义的String消息
                                                # Standard String message type defined by ROS2

"""
创建一个订阅者节点
Create a subscriber node
"""
class SubscriberNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                            # ROS2节点父类初始化
                                                                         # Initialize the ROS2 Node base class

        self.sub = self.create_subscription(\
            String, "robotum_chatter", self.listener_callback, 10)        # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
                                                                         # Create a subscriber (message type, topic name, callback function, queue size)

    def listener_callback(self, msg):                                     # 创建回调函数，执行收到话题消息后对数据的处理
                                                                         # Define the callback function to process received messages

        self.get_logger().info('I heard: "%s"' % msg.data)                # 输出日志信息，提示订阅收到的话题消息
                                                                         # Log the received topic message
        
def main(args=None):                                   # ROS2节点主入口main函数
                                                     # Main entry function for the ROS2 node

    rclpy.init(args=args)                              # ROS2 Python接口初始化
                                                     # Initialize the ROS2 Python client library

    node = SubscriberNode("topic_hellorobotum_sub")    # 创建ROS2节点对象并进行初始化
                                                     # Create and initialize the ROS2 node object

    rclpy.spin(node)                                   # 循环等待ROS2退出
                                                     # Keep the node alive and waiting for shutdown

    node.destroy_node()                                # 销毁节点对象
                                                     # Destroy the ROS2 node object

    rclpy.shutdown()                                   # 关闭ROS2 Python接口
                                                     # Shutdown the ROS2 Python client library
