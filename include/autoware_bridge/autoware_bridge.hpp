#ifndef AUTOWARE_BRIDGE_HPP
#define AUTOWARE_BRIDGE_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_bridge/localization_task.hpp"
#include "autoware_bridge/set_goal_task.hpp"

#include <autoware_bridge/srv/cancel_task.hpp>
#include <autoware_bridge/srv/get_task_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

class AutowareBridgeNode : public rclcpp::Node
{
public:
  AutowareBridgeNode();
  ~AutowareBridgeNode();

private:
  // ROS 2 Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;

  // ROS 2 Services
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;
  rclcpp::Service<autoware_bridge::srv::CancelTask>::SharedPtr cancel_service_;

  // Utility class (Now instance-based)
  AutowareBridgeUtil autoware_bridge_util_;

  // Task management
  LocalizationTask localization_task_;
  SetGoalTask set_goal_task_;

  // Task Queue and Synchronization
  std::queue<std::pair<std::string, std::function<void()>>> task_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread executor_thread_;
  bool stop_executor_ = false;

  // Private Methods
  void task_executor();
  void topic_callback_1(const std_msgs::msg::String::SharedPtr msg);
  void topic_callback_2(const std_msgs::msg::String::SharedPtr msg);

  // Service Handlers
  void handle_status_request(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response);

  void handle_cancel_request(
    const std::shared_ptr<autoware_bridge::srv::CancelTask::Request> request,
    std::shared_ptr<autoware_bridge::srv::CancelTask::Response> response);
};

#endif  // AUTOWARE_BRIDGE_HPP
