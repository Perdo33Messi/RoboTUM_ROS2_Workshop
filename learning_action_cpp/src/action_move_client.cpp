/***
@作者: Pedro
@说明: ROS2动作示例-请求执行圆周运动动作的客户端
@Author: Pedro
@Description: ROS2 Action Example - Client requesting to execute a circular motion
***/

#include <iostream>

#include "rclcpp/rclcpp.hpp"                          // ROS2 C++接口库  
                                                     // ROS2 C++ client library
#include "rclcpp_action/rclcpp_action.hpp"            // ROS2 动作类  
                                                     // ROS2 Action class
#include "learning_interface/action/move_circle.hpp"  // 自定义的圆周运动接口  
                                                     // Custom circular motion action interface

using namespace std;

class MoveCircleActionClient : public rclcpp::Node
{
    public:
        // 定义一个自定义的动作接口类，便于后续使用  
        // Define a custom action type alias for convenience
        using CustomAction = learning_interface::action::MoveCircle;

        // 定义一个处理动作请求、取消、执行的客户端类  
        // Define the goal handle type for managing goals
        using GoalHandle = rclcpp_action::ClientGoalHandle<CustomAction>;

        explicit MoveCircleActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("action_move_client", node_options)                            // ROS2节点父类初始化  
                                                                             // Initialize ROS2 node base class
        {
            this->client_ptr_ = rclcpp_action::create_client<CustomAction>(   // 创建动作客户端（接口类型、动作名）  
                                                                             // Create action client (interface type, action name)
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "move_circle");
        }

        // 创建一个发送动作目标的函数  
        // Define a function to send the action goal
        void send_goal(bool enable)
        {
            // 检查动作服务器是否可以使用  
            // Check if the action server is available
            if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
            {
                RCLCPP_ERROR(this->get_logger(), "Client: Action server not available after waiting");
                rclcpp::shutdown(); // 关闭客户端  
                                   // Shutdown the client
                return;
            }

            // 绑定动作请求、取消、执行的回调函数  
            // Bind callbacks for goal response, feedback, and result
            auto send_goal_options = rclcpp_action::Client<CustomAction>::SendGoalOptions();
            using namespace std::placeholders;
            send_goal_options.goal_response_callback =
                std::bind(&MoveCircleActionClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
                std::bind(&MoveCircleActionClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
                std::bind(&MoveCircleActionClient::result_callback, this, _1);

            // 创建一个动作目标的消息  
            // Create a goal message
            auto goal_msg = CustomAction::Goal();
            goal_msg.enable = enable;

            // 异步方式发送动作的目标  
            // Send goal asynchronously
            RCLCPP_INFO(this->get_logger(), "Client: Sending goal");
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp_action::Client<CustomAction>::SharedPtr client_ptr_;  // 动作客户端指针  
                                                                     // Action client pointer

        // 创建一个服务器收到目标之后反馈时的回调函数  
        // Callback when server responds to goal
        void goal_response_callback(GoalHandle::SharedPtr goal_message)
        {
            if (!goal_message)
            {
                RCLCPP_ERROR(this->get_logger(), "Client: Goal was rejected by server");
                rclcpp::shutdown(); // 关闭客户端节点  
                                   // Shutdown client node
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Client: Goal accepted by server, waiting for result");
            }
        }

        // 创建处理周期反馈消息的回调函数  
        // Callback for periodic feedback messages
        void feedback_callback(
            GoalHandle::SharedPtr,
            const std::shared_ptr<const CustomAction::Feedback> feedback_message)
        {
            std::stringstream ss;
            ss << "Client: Received feedback: " << feedback_message->state;
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }

        // 创建一个收到最终结果的回调函数  
        // Callback for receiving final result
        void result_callback(const GoalHandle::WrappedResult & result_message)
        {
            switch (result_message.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Client: Goal was aborted");
                    rclcpp::shutdown(); // 关闭客户端节点  
                                       // Shutdown client node
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Client: Goal was canceled");
                    rclcpp::shutdown(); // 关闭客户端节点  
                                       // Shutdown client node
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Client: Unknown result code");
                    rclcpp::shutdown(); // 关闭客户端节点  
                                       // Shutdown client node
                    return;
            }
            RCLCPP_INFO(this->get_logger(), "Client: Result received: %s", (result_message.result->finish ? "true" : "false"));
            rclcpp::shutdown();         // 关闭客户端节点  
                                       // Shutdown client node
        }
};

// ROS2节点主入口main函数  
// Main function of the ROS2 node
int main(int argc, char * argv[])                                    
{
    rclcpp::init(argc, argv);                // ROS2 C++接口初始化  
                                             // Initialize ROS2 C++ interface
    
    auto action_client = std::make_shared<MoveCircleActionClient>(); // 创建一个客户端指针  
                                                                     // Create action client instance
    
    action_client->send_goal(true);          // 发送动作目标  
                                             // Send action goal
    
    rclcpp::spin(action_client);             // 创建ROS2节点对象并进行初始化  
                                             // Spin the node to keep it alive
    
    rclcpp::shutdown();                      // 关闭ROS2 C++接口  
                                             // Shutdown the ROS2 C++ interface
    
    return 0;
}
