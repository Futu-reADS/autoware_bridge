#include "autoware_bridge/route_planning.hpp"

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>

RoutePlanning::RoutePlanning(
  rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
  std::atomic<bool> & is_task_running)
: node_(node),
  autoware_bridge_util_(autoware_bridge_util),
  cancel_requested_(false),
  is_task_running_(is_task_running),
  state_(RoutePlanningTaskState::SET_GOAL),

{
  route_state_sub_ = node_->create_subscription<RouteState>(
    "/api/routing/state", 10,
    std::bind(&RoutePlanning::routeStateCallback, this, std::placeholders::_1));
  operation_mode_state_sub_ = node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", 10,
    std::bind(&RoutePlanning::operationModeStateCallback, this, std::placeholders::_1));
  target_goal_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10);
}

void RoutePlanning::execute(
  const std::string & task_id, const geometry_msgs::msg::PoseStamped & goal_pose)
{
  autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "RUNNING");
  autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::TOTAL_RETRIES, "", 5);
  is_task_running_ = true;

  // Maximum number of initialization retries
  const int MAX_INIT_RETRIES = 5;
  int retry_counter = 0;
  bool success = false;

  while (true) {
    std::lock_guard<std::mutex> lock(task_mutex_);

    if (cancel_requested_.load()) {
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "CANCELLED");
      autoware_bridge_util_->updateTaskStatus(
        task_id, TaskRequestType::REASON, "Cancelled by user");
      RCLCPP_INFO(node_->get_logger(), "Route planning task %s cancelled.", task_id.c_str());
      is_task_running_ = false;
      return;
    }

    if (retry_counter >= MAX_INIT_RETRIES) {
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "FAILED");
      autoware_bridge_util_->updateTaskStatus(
        task_id, TaskRequestType::REASON, "Max retries elapsed");
      is_task_running_ = false;
      break;
    }

    if (success) {
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "SUCCESS");
      is_task_running_ = false;
      break;
    }

    switch (state_) {
      case RoutePlanningTaskState::SET_GOAL:

        if (retry_counter == 1) {
          autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "RETRYING");
        }
        retry_counter++;
        autoware_bridge_util_->updateTaskStatus(
          task_id, TaskRequestType::RETRIES, "", retry_counter);

        publishTargetPose(goal_pose);
        planning_start_time_ = node_->get_clock()->now();
        state_ = RoutePlanningTaskState::WAIT_FOR_AUTOWARE_ROUTE_PLANNING;
        break;

      case RoutePlanningTaskState::WAIT_FOR_AUTOWARE_ROUTE_PLANNING:
        if (
          node_->get_clock()->now().seconds() - planning_start_time_.seconds() >
          PLAN_WAIT_TIMEOUT_S) {
          RCLCPP_ERROR(node_->get_logger(), "Planning error, timeout expired");
          state_ = RoutePlanningTaskState::SET_GOAL;
          break;
        }

        switch (route_state_) {
          case RouteState::UNKNOWN:
            // Check the case
            RCLCPP_ERROR_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Planning error, Autoware in Unknown State");
            break;

          case RouteState::UNSET:
            break;

          case RouteState::SET:
            planning_start_time_ = node_->get_clock()->now();
            state_ = RoutePlanningTaskState::WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE;
            break;

          case RouteState::ARRIVED:
            autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "FAILED");
            autoware_bridge_util_->updateTaskStatus(
              task_id, TaskRequestType::REASON, "Autoware in ARRIVED State");
            RCLCPP_WARN(node_->get_logger(), "Autoware in ARRIVED State");
            break;

          case RouteState::CHANGING:
            // Check the case
            RCLCPP_ERROR(node_->get_logger(), "Autoware in CHANGING State");
            break;
        }
        break;

      case RoutePlanningTaskState::WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE:
        if (operation_mode_state_.is_autonomous_mode_available) {
          success = true;
          RCLCPP_INFO(node_->get_logger(), "AUTO mode Enabled ");
        }  // else on the basis of timer for example 5sec , Go for a retry
        else if (node_->get_clock()->now().seconds() - planning_start_time_.seconds() >= 10) {
          state_ = RoutePlanningTaskState::SET_GOAL;
          // retry_counter++;
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Autonomous operation mode is still not active... retrying planning please wait.....");
        }
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown state encountered in Route Planning.");
        break;
    }
  }
}

void RoutePlanning::request_cancel()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  cancel_requested_ = true;
}

void RoutePlanning::publishTargetPose(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  target_goal_pub_->publish(goal_pose);
}

void RoutePlanning::routeStateCallback(const RouteState msg)
{
  route_state_ = msg.state;
}

void RoutePlanning::operationModeStateCallback(const OperationModeState msg)
{
  operation_mode_state_ = msg;
}