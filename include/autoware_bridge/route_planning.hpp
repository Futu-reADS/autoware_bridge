#ifndef ROUTE_PLANNING_HPP
#define ROUTE_PLANNING_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "base_task.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>

enum class RoutePlanningTaskState {
  UNINITIALIZED,
  INITIALIZATION,
  LOCALIZATION,
  LOCALIZATION_CHECK
};

class RoutePlanning : public BaseTask
{
public:
  RoutePlanning(
    rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
    std::atomic<bool> & is_task_running);

  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose)
    override;                      // Executes SetGoal
  void request_cancel() override;  // Requests task cancellation

private:
  rclcpp::Node::SharedPtr node_;
  RoutePlanningTaskState state_;
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;  // Use shared_ptr instead of reference
  std::atomic<bool> cancel_requested_;
  std::atomic<bool> & is_task_running_;
  uint16_t route_state_;
  rclcpp::Time planning_start_time_;
  OperationModeState operation_mode_state_;
  // Subscriber
  rclcpp::Subscription<RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_pub_;
  // callback
  void route_state_sub_callback(const RouteState msg);
  void operation_mode_state_sub_callback(const OperationModeState msg);
};

#endif  // ROUTE_PLANNING_HPP
