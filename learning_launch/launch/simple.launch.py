from launch import LaunchDescription           # launch文件的描述类  
                                               # Class to describe the launch file

from launch_ros.actions import Node            # 节点启动的描述类  
                                               # Class to describe a ROS2 node launch

def generate_launch_description():             # 自动生成launch文件的函数  
                                               # Function to automatically generate the launch description
    return LaunchDescription([                 # 返回launch文件的描述信息  
                                               # Return the launch file description
        Node(                                  # 配置一个节点的启动  
                                               # Configure a node to launch
            package='learning_topic',          # 节点所在的功能包  
                                               # The package where the node is located
            executable='topic_hellorobotum_pub', # 节点的可执行文件  
                                               # The executable name of the node
        ),
        Node(                                  # 配置一个节点的启动  
                                               # Configure another node to launch
            package='learning_topic',          # 节点所在的功能包  
                                               # The package where the node is located
            executable='topic_hellorobotum_sub', # 节点的可执行文件名  
                                               # The executable name of the node
        ),
    ])
