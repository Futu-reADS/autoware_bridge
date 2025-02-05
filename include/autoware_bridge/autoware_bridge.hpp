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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;

  // Task Management
  std::unordered_map<std::string, std::string> task_status_;
  std::mutex task_mutex_;
  std::vector<std::thread> active_threads_;  // Track active threads for graceful shutdown

  // Methods
  std::string generate_task_id();
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  void handle_status_request(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response);
  void update_task_status(const std::string & task_id, const std::string & status);
  void graceful_shutdown();  // Declare graceful shutdown method
};

#endif  // AUTOWARE_BRIDGE_NODE_HPP
