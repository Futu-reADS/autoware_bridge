#ifndef ROUTE_PLANNING_HPP
#define ROUTE_PLANNING_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "base_task.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <atomic>
#include <memory>
#include <mutex>

const double ROUTE_PLANNING_TIMEOUT_S = 10.0;
const int MAX_ROUTE_PLANNING_RETRIES = 2;

enum class RoutePlanningTaskState {
  SET_GOAL,
  WAIT_FOR_AUTOWARE_ROUTE_PLANNING,
  WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE
};

class RoutePlanning : public BaseTask
{
public:
  RoutePlanning(
    rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util);

  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose)
    override;              // Executes SetGoal
  void cancel() override;  // Requests task cancellation

  // Alias for message types
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;
  std::atomic<bool> is_cancel_requested_;

  RoutePlanningTaskState state_;
  uint16_t route_state_;
  OperationModeState operation_mode_state_;

  rclcpp::Time route_planning_start_time_;
  rclcpp::Time route_state_SET_start_time_;
  std::mutex task_mutex_;

  // Subscriber
  rclcpp::Subscription<RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_state_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_pub_;

  // callback
  void routeStateCallback(const RouteState & msg);
  void operationModeStateCallback(const OperationModeState & msg);

  // Helper methods
  void publishTargetPose(const geometry_msgs::msg::PoseStamped & goal_pose);
};

#endif  // ROUTE_PLANNING_HPP
