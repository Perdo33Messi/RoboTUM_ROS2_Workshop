/***
@作者 (Author): Pedro
@说明 (Description): ROS2节点示例-发布“Hello RoboTUM”日志信息，使用面向对象的实现方式
                    Example of a ROS2 node publishing "Hello RoboTUM" log message using an object-oriented approach
***/

#include "rclcpp/rclcpp.hpp"

/***
创建一个HelloRoboTUM节点，初始化时输出“Hello RoboTUM”日志
Create a HelloRoboTUM node that outputs "Hello RoboTUM" log upon initialization
***/
class HelloRoboTUMNode : public rclcpp::Node
{
    public:
        HelloRoboTUMNode()
        : Node("node_hellorobotum_class")                          // ROS2节点父类初始化 (ROS2 Node superclass initialization)
        {
            while(rclcpp::ok())                                     // 检查ROS2系统是否正常运行 (Check if ROS2 system is running normally)
            {
                RCLCPP_INFO(this->get_logger(), "Hello RoboTUM 2025!"); // ROS2日志输出 (ROS2 log output)
                sleep(1);                                           // 休眠1秒以控制循环频率 (Sleep for 1 second to control loop frequency)
            }
        }
};

// ROS2节点主入口main函数 (Main entry function for ROS2 node)
int main(int argc, char * argv[])                               
{

    // ROS2 C++接口初始化 (Initialize ROS2 C++ interface)
    rclcpp::init(argc, argv);        
    
    // 创建ROS2节点对象并进行初始化 (Create and initialize ROS2 node object)                
    rclcpp::spin(std::make_shared<HelloRoboTUMNode>()); 
    
    // 关闭ROS2 C++接口 (Shutdown ROS2 C++ interface)
    rclcpp::shutdown();                               
    
    return 0;
}