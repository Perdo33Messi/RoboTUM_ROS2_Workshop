#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2话题示例-发布“Hello RoboTUM 2025!”话题
@Author: Pedro
@Description: ROS2 Topic Example - Publish "Hello RoboTUM 2025!" to a topic
"""

import rclpy                                     # ROS2 Python接口库
                                                # ROS2 Python client library

from rclpy.node import Node                      # ROS2 节点类
                                                # ROS2 Node class

from std_msgs.msg import String                  # 字符串消息类型
                                                # String message type

"""
创建一个发布者节点
Create a publisher node
"""
class PublisherNode(Node):  # inherit
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
                                                                 # Initialize the ROS2 Node base class
        self.pub = self.create_publisher(String, "robotum_chatter", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
                                                                 # Create a publisher (message type, topic name, queue size)

        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
                                                                 # Create a timer (interval in seconds, callback function)

    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
                                                                 # Define the callback function to be called periodically

        msg = String()                                            # 创建一个String类型的消息对象
                                                                 # Create a String message object

        msg.data = 'Hello RoboTUM 2025!'                          # 填充消息对象中的消息数据
                                                                 # Set the data field of the message

        self.pub.publish(msg)                                     # 发布话题消息
                                                                 # Publish the message to the topic

        self.get_logger().info('Publishing: "%s"' % msg.data)     # 输出日志信息，提示已经完成话题发布
                                                                 # Log information indicating the message was published
        
def main(args=None):                                   # ROS2节点主入口main函数
                                                     # Main entry point for the ROS2 node

    rclpy.init(args=args)                              # ROS2 Python接口初始化
                                                     # Initialize the ROS2 Python client library

    node = PublisherNode("topic_hellorobotum_pub")     # 创建ROS2节点对象并进行初始化
                                                     # Create and initialize the ROS2 node object

    rclpy.spin(node)                                   # 循环等待ROS2退出
                                                     # Keep the node alive and processing callbacks

    node.destroy_node()                                # 销毁节点对象
                                                     # Destroy the node object

    rclpy.shutdown()                                   # 关闭ROS2 Python接口
                                                     # Shutdown the ROS2 Python client library
