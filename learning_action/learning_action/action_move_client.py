#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: Pedro
@说明: ROS2动作示例-请求执行圆周运动动作的客户端
@Author: Pedro
@Description: ROS2 Action Example - Client to request execution of circular motion action
"""

import rclpy                                     # ROS2 Python接口库  
                                                 # ROS2 Python client library

from rclpy.node import Node                      # ROS2 节点类  
                                                 # ROS2 Node class

from rclpy.action import ActionClient            # ROS2 动作客户端类  
                                                 # ROS2 Action client class

from learning_interface.action import MoveCircle  # 自定义的圆周运动接口  
                                                 # Custom action interface for circular motion

class MoveCircleActionClient(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化  
                                                 # Initialize ROS2 Node base class

        self._action_client = ActionClient(      # 创建动作客户端（接口类型、动作名）  
                                                 # Create action client (interface type, action name)
            self, MoveCircle, 'move_circle') 

    def send_goal(self, enable):                 # 创建一个发送动作目标的函数  
                                                 # Define a function to send action goal
        goal_msg = MoveCircle.Goal()             # 创建一个动作目标的消息  
                                                 # Create a goal message for the action

        goal_msg.enable = enable                 # 设置动作目标为使能，希望机器人开始运动  
                                                 # Set goal: enable robot to start moving

        self._action_client.wait_for_server()    # 等待动作的服务器端启动  
                                                 # Wait for the action server to be available

        self._send_goal_future = self._action_client.send_goal_async(   # 异步方式发送动作的目标  
                                                                        # Send the goal asynchronously
            goal_msg,                                                   # 动作目标  
                                                                        # Action goal
            feedback_callback=self.feedback_callback)                   # 处理周期反馈消息的回调函数  
                                                                        # Callback for periodic feedback
                          
        self._send_goal_future.add_done_callback(self.goal_response_callback) # 设置一个服务器收到目标之后反馈时的回调函数  
                                                                              # Set callback for goal response
#
# 调用 send_goal(True) 向 move_circle 动作服务器发送“开始运动”目标

# 异步监听：
# 【1】动作是否被接受（goal_response_callback）
# 【2】动作执行过程中的反馈（feedback_callback）
# 【3】最终是否完成以及结果（get_result_callback）

#
# Call send_goal(True) to send a "start motion" goal to the move_circle action server

# Asynchronous callbacks:
# [1] Whether the goal is accepted (goal_response_callback)
# [2] Feedback during the execution of the action (feedback_callback)
# [3] Whether the action is completed and the final result (get_result_callback)

    def goal_response_callback(self, future):           # 创建一个服务器收到目标之后反馈时的回调函数  
                                                        # Callback after server receives goal
        goal_handle = future.result()                   # 接收动作的结果  
                                                        # Get the goal handle
        if not goal_handle.accepted:                    # 如果动作被拒绝执行  
                                                        # If goal was rejected
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')      # 动作被顺利执行  
                                                        # Goal accepted successfully

        self._get_result_future = goal_handle.get_result_async()              # 异步获取动作最终执行的结果反馈  
                                                                              # Asynchronously get final result
        self._get_result_future.add_done_callback(self.get_result_callback)   # 设置一个收到最终结果的回调函数  
                                                                              # Set callback to handle final result

    def get_result_callback(self, future):              # 创建一个收到最终结果的回调函数  
                                                        # Callback when final result is received
        result = future.result().result                 # 读取动作执行的结果  
                                                        # Get the result from future
        self.get_logger().info('Result: {%d}' % result.finish)  # 日志输出执行结果  
                                                                # Log the result

    def feedback_callback(self, feedback_msg):          # 创建处理周期反馈消息的回调函数  
                                                        # Callback for periodic feedback
        feedback = feedback_msg.feedback                # 读取反馈的数据  
                                                        # Extract feedback data
        self.get_logger().info('Received feedback: {%d}' % feedback.state)  # 输出反馈状态  
                                                                            # Log feedback status

def main(args=None):                                       # ROS2节点主入口main函数  
                                                           # Main function for ROS2 node
    rclpy.init(args=args)                                  # ROS2 Python接口初始化  
                                                           # Initialize ROS2 Python client library

    node = MoveCircleActionClient("action_move_client")    # 创建ROS2节点对象并进行初始化  
                                                           # Create and initialize the ROS2 node

    node.send_goal(True)                                   # 发送动作目标  
                                                           # Send action goal (enable = True)

    rclpy.spin(node)                                       # 循环等待ROS2退出  
                                                           # Keep spinning until ROS2 shuts down

    node.destroy_node()                                    # 销毁节点对象  
                                                           # Destroy the node

    rclpy.shutdown()                                       # 关闭ROS2 Python接口  
                                                           # Shutdown the ROS2 client library
