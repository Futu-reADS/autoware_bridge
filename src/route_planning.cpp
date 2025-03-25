#include "autoware_bridge/route_planning.hpp"

RoutePlanning::RoutePlanning(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util)
    : node_(node),
      autoware_bridge_util_(autoware_bridge_util),
      is_cancel_requested_(false),
      state_(RoutePlanningTaskState::SET_GOAL),
      route_state_(RouteState::UNKNOWN),
      operation_mode_state_(),
      route_planning_start_time_(rclcpp::Time(0)),
      route_state_SET_start_time_(rclcpp::Time(0))
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
    const std::string &task_id, const geometry_msgs::msg::PoseStamped &goal_pose)
{
  autoware_bridge_util_->updateTaskStatus(task_id, "RUNNING");

  // Maximum number of route planning retries
  int retry_counter = 0;
  bool success = false;

  while (true)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);

    if (is_cancel_requested_)
    {
      // CANCEL
      autoware_bridge_util_->updateTaskStatus(task_id, "CANCELLED");
      RCLCPP_INFO(node_->get_logger(), "Route planning task %s cancelled.", task_id.c_str());
      return;
    }

    if (retry_counter >= MAX_ROUTE_PLANNING_RETRIES)
    {
      // FAILURE
      autoware_bridge_util_->updateTaskStatus(task_id, "FAILED", "Max retries elapsed");
      RCLCPP_ERROR(node_->get_logger(), "Route planning task %s failed.", task_id.c_str());
      break;
    }

    if (success)
    {
      // SUCCESS
      autoware_bridge_util_->updateSuccessStatus(task_id);
      RCLCPP_INFO(node_->get_logger(), "Route planning task %s successful.", task_id.c_str());
      break;
    }

    /*https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/routing/
    UNSET (the route is not set; effectively “waiting” for a route request)
    SET (the route is set)
    CHANGING (trying to change the route)
    ARRIVED (the vehicle has arrived at the destination)

    Matching the Diagram to the Table
    The diagram’s “WAITING” is effectively the table’s “UNSET” state.
    After the route is set, you go from UNSET → SET.
    Then if you change the route, you go from SET → CHANGING.
    Finally, once you arrive or finish changing successfully, you can go to ARRIVED (or back to SET
    if you are reusing the route).

    Basically “WAITING” = “UNSET”

    */
    switch (state_)
    {
    case RoutePlanningTaskState::SET_GOAL:

      if (retry_counter == 1)
      {
        autoware_bridge_util_->updateTaskStatus(task_id, "RETRYING");
      }
      retry_counter++;
      autoware_bridge_util_->updateTaskRetries(task_id, retry_counter);

      publishTargetPose(goal_pose);
      route_planning_start_time_ = node_->get_clock()->now();
      state_ = RoutePlanningTaskState::WAIT_FOR_AUTOWARE_ROUTE_PLANNING;
      break;

    case RoutePlanningTaskState::WAIT_FOR_AUTOWARE_ROUTE_PLANNING:
      if (
          node_->get_clock()->now().seconds() - route_planning_start_time_.seconds() >
          ROUTE_PLANNING_TIMEOUT_S)
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning error, timeout expired");
        state_ = RoutePlanningTaskState::SET_GOAL;
        break;
      }

      switch (route_state_)
      {
      case RouteState::UNKNOWN:
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Planning error, Autoware in Unknown State");
        break;

      case RouteState::UNSET:
        break;

      case RouteState::SET:
        route_state_SET_start_time_ = node_->get_clock()->now();
        state_ = RoutePlanningTaskState::WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE;
        break;

      case RouteState::ARRIVED:
        autoware_bridge_util_->updateTaskStatus(task_id, "FAILED", "Autoware in ARRIVED State");
        RCLCPP_WARN(node_->get_logger(), "Autoware in ARRIVED State");
        break;

      case RouteState::CHANGING:
        RCLCPP_ERROR(node_->get_logger(), "Autoware in CHANGING State");
        break;
      }
      break;

    case RoutePlanningTaskState::WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE:
      if (operation_mode_state_.is_autonomous_mode_available)
      {
        success = true;
        retry_counter = 0;
        RCLCPP_INFO(node_->get_logger(), "AUTO mode Enabled ");
      }
      else if (
          node_->get_clock()->now().seconds() - route_state_SET_start_time_.seconds() >= 10)
      {
        state_ = RoutePlanningTaskState::SET_GOAL;
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

void RoutePlanning::cancel()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  is_cancel_requested_ = true;
}

void RoutePlanning::publishTargetPose(const geometry_msgs::msg::PoseStamped &goal_pose)
{
  target_goal_pub_->publish(goal_pose);
}

void RoutePlanning::routeStateCallback(const RouteState &msg)
{
  route_state_ = msg.state;
}

void RoutePlanning::operationModeStateCallback(const OperationModeState &msg)
{
  operation_mode_state_ = msg;
}