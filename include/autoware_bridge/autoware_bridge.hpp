#ifndef AUTOWARE_BRIDGE_HPP
#define AUTOWARE_BRIDGE_HPP

#include "autoware_bridge/autonomous_driving.hpp"
#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_bridge/localization.hpp"
#include "autoware_bridge/route_planning.hpp"

#include <autoware_bridge/srv/get_task_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_bridge_msgs/msg/task_status_response.hpp"
#include "ftd_master_msgs/msg/pose_stamped_with_task_id.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <memory>
#include <thread>

class AutowareBridgeNode : public rclcpp::Node
{
public:
  AutowareBridgeNode(
      std::shared_ptr<AutowareBridgeUtil> util,
      std::shared_ptr<Localization> localization_task,
      std::shared_ptr<RoutePlanning> route_planning_task,
      std::shared_ptr<AutonomousDriving> autonomous_driving_task,
      std::atomic<bool> &is_task_running);

  ~AutowareBridgeNode();

private:
  // ROS2 Subscriptions
  rclcpp::Subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>::SharedPtr
      localization_request_subscription_;
  rclcpp::Subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>::SharedPtr
      route_planning_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomous_driving_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_task_subscription_;

  // ROS2 Publishers
  rclcpp::Publisher<autoware_bridge_msgs::msg::TaskStatusResponse>::SharedPtr
      task_response_publisher_;
  rclcpp::Publisher<autoware_bridge_msgs::msg::TaskStatusResponse>::SharedPtr
      cancel_response_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reinitialize_response_publisher_;

  // ROS2 Services
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;

  // Shared Utility instances and task instances
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;
  // std::shared_ptr<Localization> localization_task_;
  // std::shared_ptr<RoutePlanning> route_planning_task_;
  // std::shared_ptr<AutonomousDriving> autonomous_driving_task_;
  std::atomic<bool> &is_task_running_; // Shared atomic flag

  // Callback Methods
  void localizationRequestCallback(const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg);
  void routePlanningRequestCallback(const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg);
  void autonomousDrivingRequestCallback(const std_msgs::msg::String::SharedPtr msg);
  void cancelTaskCallback(const std_msgs::msg::String::SharedPtr msg);
  void onTimerCallback();
  void publish_relocalization_notification(const bool localization_quality);
  template <typename TaskType>

  // Helper functions
  bool isTaskRejected(const std::string &task_name);
  void startTaskExecution(
      const std::string &requested_task_id, const geometry_msgs::msg::PoseStamped &pose_stamped,
      std::shared_ptr<BaseTask> task);
  void startThreadExecution(
      const std::string &requested_task_id, const geometry_msgs::msg::PoseStamped &pose_stamped);
  void publishTaskRejectionReason(const std::string &task_name);
  void publishTaskResponse(const std::string &task_id);
  void publishCancelResponse(const std::string &task_id);

  autoware_bridge_msgs::msg::TaskStatusResponse createTaskStatusResponse(
      const std::string &task_id, const std::string &status, const std::string &reason);

  // Service Handlers
  void handleStatusRequest(
      const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
      std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response);

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // AUTOWARE_BRIDGE_HPP
