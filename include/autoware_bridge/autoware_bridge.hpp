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
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <atomic>
#include <memory>
#include <mutex>

class AutowareBridgeNode : public rclcpp::Node
{
public:
  AutowareBridgeNode(std::shared_ptr<AutowareBridgeUtil> util);

  ~AutowareBridgeNode();

  // Alias for message types
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;

private:
  // ROS2 Subscriptions
  rclcpp::Subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>::SharedPtr
    localization_request_subscription_;
  rclcpp::Subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>::SharedPtr
    route_planning_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomous_driving_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_task_subscription_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr localization_quality_subscriber_;

  // ROS2 Publishers
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr task_response_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr cancel_response_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reinitialize_response_publisher_;

  // ROS2 Services
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;

  // Shared Utility instance
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;

  bool is_cancel_requested_;
  bool localization_quality_;
  std::mutex task_mutex_;

  // Callback Methods
  void localizationRequestCallback(
    const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg);
  void routePlanningRequestCallback(
    const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg);
  void autonomousDrivingRequestCallback(const std_msgs::msg::String::SharedPtr msg);
  void cancelTaskCallback(const std_msgs::msg::String::SharedPtr msg);
  void onTimerCallback();
  void localizationQualityCallback(const ModeChangeAvailable & msg);

  // Helper functions
  bool isTaskRejected(const std::string & task_name);
  void startTaskExecution(
    const std::string & requested_task_id, const geometry_msgs::msg::PoseStamped & pose_stamped,
    std::shared_ptr<BaseTask> task);
  void startThreadExecution(
    const std::string & requested_task_id, const geometry_msgs::msg::PoseStamped & pose_stamped);
  void publishTaskRejectionReason(
    const std::string & task_name, const std::string & active_task_id);
  void publishTaskResponse(const std::string & task_id);
  void publishCancelResponse(const std::string & task_id);

  // Service Handlers
  void handleStatusRequestSrvc(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response);

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // AUTOWARE_BRIDGE_HPP
