#ifndef AUTOWARE_BRIDGE_HPP
#define AUTOWARE_BRIDGE_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_bridge/driving_task.hpp"
#include "autoware_bridge/localization_task.hpp"
#include "autoware_bridge/set_goal_task.hpp"

#include <autoware_bridge/srv/cancel_task.hpp>
#include <autoware_bridge/srv/get_task_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <mutex>
#include <thread>

class AutowareBridgeNode : public rclcpp::Node
{
public:
  AutowareBridgeNode();
  ~AutowareBridgeNode();

private:
  // ROS2 Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_3_;

  // ROS2 task status Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_rejection_status_publisher_;

  // ROS2 Services
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;
  rclcpp::Service<autoware_bridge::srv::CancelTask>::SharedPtr cancel_service_;

  // Utility class instance
  AutowareBridgeUtil autoware_bridge_util_;

  // Task management
  LocalizationTask localization_task_;
  SetGoalTask set_goal_task_;
  DrivingTask driving_task_;

  // Private Methods
  void topic_callback_1(const std_msgs::msg::String::SharedPtr msg);
  void topic_callback_2(const std_msgs::msg::String::SharedPtr msg);
  void topic_callback_3(const std_msgs::msg::String::SharedPtr msg);
  void publish_task_rejection_status(const std::string & task_name);

  // Service Handlers
  void handle_status_request(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response);

  void handle_cancel_request(
    const std::shared_ptr<autoware_bridge::srv::CancelTask::Request> request,
    std::shared_ptr<autoware_bridge::srv::CancelTask::Response> response);

  std::atomic<bool> is_task_running_;
};

#endif  // AUTOWARE_BRIDGE_HPP
