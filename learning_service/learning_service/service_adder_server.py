#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2服务示例-提供加法器的服务器处理功能
@Author: Pedro
@Description: ROS2 Service Example - Provide server-side processing for an adder
"""

import rclpy                                     # ROS2 Python接口库
                                                # ROS2 Python client library

from rclpy.node import Node                     # ROS2 节点类
                                                # ROS2 Node class

from learning_interface.srv import AddTwoInts    # 自定义的服务接口
                                                # Custom service interface

class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)                                                             # ROS2节点父类初始化
                                                                                          # Initialize the ROS2 Node base class

        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.adder_callback)    # 创建服务器对象（接口类型、服务名、服务器回调函数）
                                                                                          # Create service (interface type, service name, callback function)

        # create_service 是 ROS2 中用来创建 服务（Service）的一个函数调用。具体来说：
        # create_service is a ROS2 function used to create a service. Specifically:
        # self.create_service(服务接口类型, 服务名称, 回调函数)
        # self.create_service(service type, service name, callback function)

    def adder_callback(self, request, response):                                           # 创建回调函数，执行收到请求后对数据的处理
                                                                                          # Define callback function to process incoming requests

        response.sum = request.a + request.b                                               # 完成加法求和计算，将结果放到反馈的数据中
                                                                                          # Perform addition and assign result to response

        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))   # 输出日志信息，提示已经完成加法求和计算
                                                                                          # Log the request details for verification

        return response                                                                    # 反馈应答信息
                                                                                          # Return the response

def main(args=None):                                 # ROS2节点主入口main函数
                                                    # Main entry function for ROS2 node

    rclpy.init(args=args)                            # ROS2 Python接口初始化
                                                    # Initialize the ROS2 Python client library

    node = adderServer("service_adder_server")       # 创建ROS2节点对象并进行初始化
                                                    # Create and initialize the ROS2 node object

    rclpy.spin(node)                                 # 循环等待ROS2退出
                                                    # Keep the node alive and processing callbacks

    node.destroy_node()                              # 销毁节点对象
                                                    # Destroy the node object

    rclpy.shutdown()                                 # 关闭ROS2 Python接口
                                                    # Shutdown the ROS2 Python client library
