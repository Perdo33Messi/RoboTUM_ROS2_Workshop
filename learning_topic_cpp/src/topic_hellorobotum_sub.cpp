/***
@作者: Pedro
@说明: ROS2话题示例-订阅“Hello RoboTUM”话题消息
@Author: Pedro
@Description: ROS2 Topic Example - Subscribe to the "Hello RoboTUM" topic message
***/

#include <memory>

#include "rclcpp/rclcpp.hpp"                  // ROS2 C++接口库
                                              // ROS2 C++ client library
#include "std_msgs/msg/string.hpp"            // 字符串消息类型
                                              // String message type

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
    public:
        SubscriberNode()
        : Node("topic_hellorobotum_sub")        // ROS2节点父类初始化
                                               // Initialize ROS2 node base class
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(       
                "robotum_chatter", 10, std::bind(&SubscriberNode::topic_callback, this, _1));   
                                                  // 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
                                                  // Create subscriber (message type, topic name, callback function, queue size)
        }

    private:
        void topic_callback(const std_msgs::msg::String & msg) const                  
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());       
                                                  // 输出日志信息，提示订阅收到的话题消息
                                                  // Log output showing the received topic message
        }
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;         
                                                  // 订阅者指针
                                                  // Subscriber pointer
};

// ROS2节点主入口main函数
// Main entry function for ROS2 node
int main(int argc, char * argv[])                         
{
    rclcpp::init(argc, argv);                 
    // ROS2 C++接口初始化
    // Initialize ROS2 C++ client library
    
    rclcpp::spin(std::make_shared<SubscriberNode>());     
    // 创建ROS2节点对象并进行初始化            
    // Create and run ROS2 node object
    
    rclcpp::shutdown();                                  
    // 关闭ROS2 C++接口
    // Shutdown ROS2 C++ client library
    
    return 0;
}