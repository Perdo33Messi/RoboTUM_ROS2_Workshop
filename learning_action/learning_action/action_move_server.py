#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2动作示例-负责执行圆周运动动作的服务端
@Author: Pedro
@Description: ROS2 Action Example - Server responsible for executing circular motion
"""

import time

import rclpy                                      # ROS2 Python接口库  
                                                 # ROS2 Python client library

from rclpy.node import Node                      # ROS2 节点类  
                                                 # ROS2 Node class

from rclpy.action import ActionServer             # ROS2 动作服务器类  
                                                 # ROS2 Action server class

from learning_interface.action import MoveCircle  # 自定义的圆周运动接口  
                                                 # Custom action interface for circular motion

class MoveCircleActionServer(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化  
                                                 # Initialize ROS2 Node base class

        self._action_server = ActionServer(      # 创建动作服务器（接口类型、动作名、回调函数）  
                                                 # Create action server (interface type, action name, callback)
            self,
            MoveCircle,
            'move_circle',
            self.execute_callback)

    def execute_callback(self, goal_handle):            # 执行收到动作目标之后的处理函数  
                                                        # Callback function executed when goal is received
        self.get_logger().info('Moving circle...')      # 输出日志信息  
                                                        # Log message

        feedback_msg = MoveCircle.Feedback()            # 创建一个动作反馈信息的消息  
                                                        # Create feedback message

        for i in range(0, 360, 30):                     # 从0到360度，执行圆周运动，并周期反馈信息  
                                                        # Simulate circular motion from 0 to 360 degrees with feedback
            feedback_msg.state = i                      # 创建反馈信息，表示当前执行到的角度  
                                                        # Set feedback to current angle
            self.get_logger().info('Publishing feedback: %d' % feedback_msg.state)  
                                                        # 输出当前反馈状态  
                                                        # Log current feedback

            goal_handle.publish_feedback(feedback_msg)  # 发布反馈信息  
                                                        # Publish feedback

            time.sleep(0.5)                             # 暂停0.5秒，模拟动作执行  
                                                        # Sleep 0.5 seconds to simulate execution

        goal_handle.succeed()                           # 动作执行成功  
                                                        # Mark the goal as succeeded

        result = MoveCircle.Result()                    # 创建结果消息  
                                                        # Create result message

        result.finish = True                            # 设置执行完成标志  
                                                        # Set completion flag to True

        return result                                   # 反馈最终动作执行的结果  
                                                        # Return the final action result

def main(args=None):                                       # ROS2节点主入口main函数  
                                                           # Main function for the ROS2 node
    rclpy.init(args=args)                                  # ROS2 Python接口初始化  
                                                           # Initialize ROS2 Python client library

    node = MoveCircleActionServer("action_move_server")    # 创建ROS2节点对象并进行初始化  
                                                           # Create and initialize ROS2 node

    rclpy.spin(node)                                       # 循环等待ROS2退出  
                                                           # Spin the node and wait for shutdown

    node.destroy_node()                                    # 销毁节点对象  
                                                           # Destroy the node object

    rclpy.shutdown()                                       # 关闭ROS2 Python接口  
                                                           # Shutdown the ROS2 Python client library
