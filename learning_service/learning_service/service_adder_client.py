#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2服务示例-发送两个加数，请求加法器计算
@Author: Pedro
@Description: ROS2 Service Example - Send two addends to a service and request the adder to calculate the sum
"""

import sys

import rclpy                                                    # ROS2 Python接口库
                                                                # ROS2 Python client library

from rclpy.node import Node                                     # ROS2 节点类
                                                                # ROS2 Node class

from learning_interface.srv import AddTwoInts                   # 自定义的服务接口
                                                                # Custom service interface

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                    # ROS2节点父类初始化
                                                                                 # Initialize the ROS2 Node base class

        self.client = self.create_client(AddTwoInts, 'add_two_ints')              # 创建服务客户端对象（服务接口类型，服务名）
                                                                                 # Create the service client (service type, service name)

        while not self.client.wait_for_service(timeout_sec=1.0):                  # 循环等待服务器端成功启动
                                                                                 # Loop until the service server is available
            self.get_logger().info('service not available, waiting again...') 

        self.request = AddTwoInts.Request()                                       # 创建服务请求的数据对象
                                                                                 # Create a request object for the service
                    
    def send_request(self):                                                       # 创建一个发送服务请求的函数
                                                                                 # Define a function to send the service request

        self.request.a = int(sys.argv[1])                                         # 设置请求数据a
                                                                                 # Set request field 'a'

        self.request.b = int(sys.argv[2])                                         # 设置请求数据b
                                                                                 # Set request field 'b'

        self.future = self.client.call_async(self.request)                        # 异步方式发送服务请求
                                                                                 # Asynchronously send the service request

def main(args=None):
    rclpy.init(args=args)                                                         # ROS2 Python接口初始化
                                                                                 # Initialize the ROS2 Python client library

    node = adderClient("service_adder_client")                                    # 创建ROS2节点对象并进行初始化
                                                                                 # Create and initialize the ROS2 node object

    node.send_request()                                                           # 发送服务请求
                                                                                 # Send the service request
    
    while rclpy.ok():                                                             # ROS2系统正常运行
                                                                                 # Check if ROS2 system is running
        rclpy.spin_once(node)                                                     # 循环执行一次节点
                                                                                 # Execute a single cycle of the node loop

        if node.future.done():                                                    # 数据是否处理完成
                                                                                 # Check if the response has been received
            try:
                response = node.future.result()                                   # 接收服务器端的反馈数据
                                                                                 # Receive the result from the service
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))                              # 打印服务调用失败信息
                                                                                 # Log service call failure
            else:
                node.get_logger().info(                                           # 将收到的反馈信息打印输出
                    'Result of add_two_ints: for %d + %d = %d' % 
                    (node.request.a, node.request.b, response.sum))              # Print the received result
            break
            
    node.destroy_node()                                                           # 销毁节点对象
                                                                                 # Destroy the ROS2 node object

    rclpy.shutdown()                                                              # 关闭ROS2 Python接口
                                                                                 # Shutdown the ROS2 Python client library
