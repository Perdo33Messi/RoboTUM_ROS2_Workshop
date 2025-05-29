/***
@作者: Pedro
@说明: ROS2话题示例-发布话题
@Author: Pedro
@Description: ROS2 Topic Example - Publish to a topic
***/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"          // ROS2 C++接口库
                                     // ROS2 C++ client library
#include "std_msgs/msg/string.hpp"    // 字符串消息类型
                                     // String message type

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
    public:
        PublisherNode()
        : Node("topic_hellorobotum_pub") // ROS2节点父类初始化
                                        // Initialize the ROS2 node base class
        {
            // 创建发布者对象（消息类型、话题名、队列长度）
            // Create publisher object (message type, topic name, queue size)
            publisher_ = this->create_publisher<std_msgs::msg::String>("robotum_chatter", 10); 

            // 创建一个定时器,定时执行回调函数
            // Create a timer to periodically call the callback function
            timer_ = this->create_wall_timer(
                500ms, std::bind(&PublisherNode::timer_callback, this));            
        }

    private:
        // 创建定时器周期执行的回调函数
        // Define the callback function to be called periodically by the timer
        void timer_callback()                                                       
        {
          // 创建一个String类型的消息对象
          // Create a String message object
          auto msg = std_msgs::msg::String();   

          // 填充消息对象中的消息数据
          // Fill the message data
          msg.data = "Hello RoboTUM 2025!";

          // 发布话题消息
          // Publish the message
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str()); 

          // 输出日志信息，提示已经完成话题发布   
          // Log output indicating the message was published
          publisher_->publish(msg);                                                
        }
        
        rclcpp::TimerBase::SharedPtr timer_;                             // 定时器指针
                                                                         // Timer pointer
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 发布者指针
                                                                         // Publisher pointer
};

// ROS2节点主入口main函数
// Main function of the ROS2 node
int main(int argc, char * argv[])                      
{
    rclcpp::init(argc, argv);                
    // ROS2 C++接口初始化
    // Initialize ROS2 C++ client library
    
    rclcpp::spin(std::make_shared<PublisherNode>());   
    // 创建ROS2节点对象并进行初始化          
    // Create and run the ROS2 node object
    
    rclcpp::shutdown();                                
    // 关闭ROS2 C++接口
    // Shutdown ROS2 C++ client library

    return 0;
}
