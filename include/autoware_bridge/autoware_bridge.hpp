#ifndef AUTOWARE_BRIDGE_NODE_HPP
#define AUTOWARE_BRIDGE_NODE_HPP

#include <autoware_bridge/srv/get_task_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

class AutowareBridgeNode : public rclcpp::Node
{
public:
  AutowareBridgeNode();

private:
  // ROS Interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;

  // Task Management
  std::unordered_map<std::string, std::string> task_status_;  // Track task statuses
  std::mutex task_mutex_;                                     // Ensure thread-safe access
  std::vector<std::thread> active_threads_;  // Manage active threads for graceful shutdown

  // Utility Methods
  std::string generate_task_id(const std::string & prefix);  // Generate unique task IDs
  void update_task_status(
    const std::string & task_id, const std::string & status);  // Update task status
  void graceful_shutdown();                                    // Handle graceful shutdown

  // Topic Callbacks
  void topic_callback_1(const std_msgs::msg::String::SharedPtr msg);  // Handles topic 1
  void topic_callback_2(const std_msgs::msg::String::SharedPtr msg);  // Handles topic 2

  // Task Functions
  void localization(const std::string & task_id);  // Task for topic 1
  void set_goal(const std::string & task_id);      // Task for topic 2

  // Service Handler
  void handle_status_request(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response>
      response);  // Missing declaration
};

#endif  // AUTOWARE_BRIDGE_NODE_HPP

