#ifndef AUTOWARE_BRIDGE_HPP
#define AUTOWARE_BRIDGE_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_bridge/driving_task.hpp"
#include "autoware_bridge/localization_task.hpp"
#include "autoware_bridge/set_goal_task.hpp"

#include <autoware_bridge/srv/get_task_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

// Include the custom message header
#include "autoware_bridge_msgs/msg/task_status_response.hpp"
#include "ftd_master_msgs/msg/pose_stamped_with_task_id.hpp"

class AutowareBridgeNode : public rclcpp::Node
{
public:
  AutowareBridgeNode(
    std::shared_ptr<AutowareBridgeUtil> util, std::shared_ptr<LocalizationTask> localization_task,
    std::shared_ptr<SetGoalTask> set_goal_task, std::shared_ptr<DrivingTask> driving_task);

  ~AutowareBridgeNode();

private:
  // ROS2 Subscriptions

  rclcpp::Subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>::SharedPtr
    localization_request_subscription_;
  rclcpp::Subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>::SharedPtr
    route_planning_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomous_driving_request_subscription_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_task_subscription_;

  // Publisher for task response (execution result)
  rclcpp::Publisher<autoware_bridge_msgs::msg::TaskStatusResponse>::SharedPtr
    task_response_publisher_;

  // Publisher for cancellation response
  rclcpp::Publisher<autoware_bridge_msgs::msg::TaskStatusResponse>::SharedPtr
    cancel_response_publisher_;

  // ROS2 Publisher for task rejection status
  rclcpp::Publisher<autoware_bridge_msgs::msg::TaskStatusResponse>::SharedPtr
    task_rejection_reason_publisher_;

  // ROS2 Services
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;

  // // Shared Utility instance and tasks
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;
  std::shared_ptr<LocalizationTask> localization_task_;
  std::shared_ptr<SetGoalTask> set_goal_task_;
  std::shared_ptr<DrivingTask> driving_task_;

  // Private Methods
  void localization_request_callback(
    const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg);
  void route_planning_request_callback(
    const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg);
  void autonomous_driving_request_callback(const std_msgs::msg::String::SharedPtr msg);
  void cancel_task_callback(const std_msgs::msg::String::SharedPtr msg);

  void publish_task_rejection_reason(const std::string & task_name);

  // Service Handlers
  void handle_status_request(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response);

  // Single active task tracking flag
  std::atomic<bool> is_task_running_;

  // Helper functions
  void publishTaskResponse(const std::string & task_id);
  void publishCancelResponse(const std::string & task_id);
  autoware_bridge_msgs::msg::TaskStatusResponse createTaskStatusResponse(
    const std::string & task_id);
};

#endif  // AUTOWARE_BRIDGE_HPP
