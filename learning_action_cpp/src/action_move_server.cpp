/***
@作者: Pedro
@说明: ROS2动作示例-负责执行圆周运动动作的服务端
@Author: Pedro
@Description: ROS2 Action Example - Server responsible for executing circular motion
***/

#include <iostream>

#include "rclcpp/rclcpp.hpp"                          // ROS2 C++接口库  
                                                     // ROS2 C++ client library
#include "rclcpp_action/rclcpp_action.hpp"            // ROS2 动作类  
                                                     // ROS2 Action class
#include "learning_interface/action/move_circle.hpp"  // 自定义的圆周运动接口  
                                                     // Custom circular motion action interface

using namespace std;

class MoveCircleActionServer : public rclcpp::Node
{
    public:
        // 定义一个自定义的动作接口类，便于后续使用  
        // Define a custom action interface type for easier use
        using CustomAction = learning_interface::action::MoveCircle;

        // 定义一个处理动作请求、取消、执行的服务器端  
        // Define goal handle type to manage requests, cancellation, and execution
        using GoalHandle = rclcpp_action::ServerGoalHandle<CustomAction>;

        explicit MoveCircleActionServer(const rclcpp::NodeOptions & action_server_options = rclcpp::NodeOptions())
        : Node("action_move_server", action_server_options)                                     // ROS2节点父类初始化  
                                                                                                // Initialize ROS2 node base class
        {
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<CustomAction>(                  // 创建动作服务器（接口类型、动作名、回调函数）  
                                                                                                // Create action server (interface type, name, and callbacks)
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "move_circle",
                std::bind(&MoveCircleActionServer::handle_goal, this, _1, _2),
                std::bind(&MoveCircleActionServer::handle_cancel, this, _1),
                std::bind(&MoveCircleActionServer::handle_accepted, this, _1));
        }

    private:
        rclcpp_action::Server<CustomAction>::SharedPtr action_server_;  // 动作服务器  
                                                                        // Action server

        // 响应动作目标的请求  
        // Handle incoming goal request
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const CustomAction::Goal> goal_request)                             
        {
            RCLCPP_INFO(this->get_logger(), "Server: Received goal request: %d", goal_request->enable);
            (void)uuid;

            // 如请求为enable则接受运动请求，否则就拒绝  
            // Accept if goal requests motion enabled; otherwise reject
            if (goal_request->enable)                                                           
            {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
            else
            {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // 响应动作取消的请求  
        // Handle goal cancellation request
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle_canceled_)                            
        {
            RCLCPP_INFO(this->get_logger(), "Server: Received request to cancel action");
            (void) goal_handle_canceled_; 
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // 处理动作接受后具体执行的过程  
        // Handle goal execution once accepted
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle_accepted_)          
        {
            using namespace std::placeholders;
            // 在线程中执行动作过程  
            // Execute goal in a separate thread
            std::thread{std::bind(&MoveCircleActionServer::execute, this, _1), goal_handle_accepted_}.detach(); 
        }

        // 动作执行逻辑  
        // Execution logic for the action
        void execute(const std::shared_ptr<GoalHandle> goal_handle_)
        {
            const auto requested_goal = goal_handle_->get_goal();       // 动作目标  
                                                                         // Action goal data
            auto feedback = std::make_shared<CustomAction::Feedback>(); // 动作反馈  
                                                                         // Action feedback object
            auto result = std::make_shared<CustomAction::Result>();     // 动作结果  
                                                                         // Action result object

            RCLCPP_INFO(this->get_logger(), "Server: Executing goal");
            rclcpp::Rate loop_rate(1);  // 控制循环频率为1Hz  
                                        // Loop at 1Hz rate

            // 动作执行的过程  
            // Execute simulated circular motion
            for (int i = 0; (i < 361) && rclcpp::ok(); i = i + 30)
            {
                // 检查是否取消动作  
                // Check if goal is being canceled
                if (goal_handle_->is_canceling())
                {
                    result->finish = false;
                    goal_handle_->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Server: Goal canceled");
                    return;
                }

                // 更新反馈状态  
                // Update feedback state
                feedback->state = i;
                // 发布反馈状态  
                // Publish feedback
                goal_handle_->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Server: Publish feedback");

                loop_rate.sleep();  // 休眠一段时间以控制节奏  
                                   // Sleep to maintain loop rate
            }

            // 动作执行完成  
            // Goal completed successfully
            if (rclcpp::ok())
            {
                result->finish = true;
                goal_handle_->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Server: Goal succeeded");
            }
        }
};

// ROS2节点主入口main函数  
// Main function for the ROS2 node
int main(int argc, char * argv[])                                
{
    rclcpp::init(argc, argv);                        // ROS2 C++接口初始化  
                                                     // Initialize ROS2 C++ client library
    
    rclcpp::spin(std::make_shared<MoveCircleActionServer>());   // 创建ROS2节点对象并进行初始化  
                                                                 // Create and spin the action server node
    
    rclcpp::shutdown();                                            // 关闭ROS2 C++接口  
                                                                   // Shutdown ROS2 C++ client library
    
    return 0;
}