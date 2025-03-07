#include "autoware_bridge/autoware_bridge.hpp"

#include <functional>
#include <thread>

AutowareBridgeNode::AutowareBridgeNode(
  std::shared_ptr<AutowareBridgeUtil> util, std::shared_ptr<Localization> localization_task,
  std::shared_ptr<RoutePlanning> route_planning_task,
  std::shared_ptr<AutonomousDriving> autonomous_driving_task)
: Node("autoware_bridge_node"),
  autoware_bridge_util_(util),
  localization_task_(localization_task),
  route_planning_task_(route_planning_task),
  autonomous_driving_task_(autonomous_driving_task),
  is_task_running_(false)
{
  // ROS2 Subscriptions
  localization_request_subscription_ =
    this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(
      "/ftd_master/localization_request", 10,
      std::bind(&AutowareBridgeNode::localizationRequestCallback, this, std::placeholders::_1));

  route_planning_request_subscription_ =
    this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(
      "/ftd_master/route_planning_request", 10,
      std::bind(&AutowareBridgeNode::routePlanningRequestCallback, this, std::placeholders::_1));

  autonomous_driving_request_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/ftd_master/autonomous_driving_request", 10,
    std::bind(&AutowareBridgeNode::autonomousDrivingRequestCallback, this, std::placeholders::_1));

  cancel_task_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/ftd_master/cancel_task", 10,
    std::bind(&AutowareBridgeNode::cancelTaskCallback, this, std::placeholders::_1));

  // ROS2 Publishers
  task_response_publisher_ = this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
    "/autoware_bridge/task_response", 10);

  cancel_response_publisher_ =
    this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
      "/autoware_bridge/cancel_task_response", 10);

  // ROS2 Services
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
    "check_task_status", std::bind(
                           &AutowareBridgeNode::handleStatusRequest, this, std::placeholders::_1,
                           std::placeholders::_2));
}

AutowareBridgeNode::~AutowareBridgeNode()
{
  // Optionally: join or signal threads if you decide to track them instead of detaching.
}

// Task Handling Callbacks

void AutowareBridgeNode::localizationRequestCallback(
  const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{
  if (isTaskRejected("localization")) {
    return;
  }
  startTaskExecution(msg->task_id.data, msg->pose_stamped, localization_task_);
}

void AutowareBridgeNode::routePlanningRequestCallback(
  const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{
  if (isTaskRejected("route_planning")) {
    return;
  }
  startTaskExecution(msg->task_id.data, msg->pose_stamped, route_planning_task_);
}

void AutowareBridgeNode::autonomousDrivingRequestCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (isTaskRejected("autonomous_driving")) {
    return;
  }
  geometry_msgs::msg::PoseStamped dummy_pose_stamped;
  startTaskExecution(msg->data, dummy_pose_stamped, autonomous_driving_task_);
}

bool AutowareBridgeNode::isTaskRejected(const std::string & task_name)
{
  if (is_task_running_.exchange(true)) {
    RCLCPP_WARN(
      this->get_logger(), "A task is already running. %s request rejected.", task_name.c_str());
    publishTaskRejectionReason(task_name);
    return true;
  }
  return false;
}

void AutowareBridgeNode::startTaskExecution(
  const std::string & requested_task_id, const geometry_msgs::msg::PoseStamped & pose_stamped,
  std::shared_ptr<BaseTask> task)
{
  RCLCPP_INFO(this->get_logger(), "Start task_id: %s", requested_task_id.c_str());
  autoware_bridge_util_->updateTaskStatus(requested_task_id, TaskRequestType::STATUS, "PENDING");
  autoware_bridge_util_->setActiveTask(task);
  startThreadExecution(requested_task_id, pose_stamped);
}

void AutowareBridgeNode::startThreadExecution(
  const std::string & requested_task_id, const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->getActiveTaskPointer();
  if (active_task) {
    std::thread([this, active_task, requested_task_id, pose_stamped]() {
      active_task->execute(requested_task_id, pose_stamped);
      autoware_bridge_util_->clearActiveTask();
      publishTaskResponse(requested_task_id);
    }).detach();
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Active task pointer is null for task_id: %s", requested_task_id.c_str());
  }
}

void AutowareBridgeNode::publishTaskRejectionReason(const std::string & task_name)
{
  std::string active_task_id = autoware_bridge_util_->getActiveTaskId();
  if (active_task_id == "NO_ACTIVE_TASK") {
    RCLCPP_WARN(
      this->get_logger(), "No active task is currently running. Ignoring %s request.",
      task_name.c_str());
    return;
  } else {
    autoware_bridge_msgs::msg::TaskStatusResponse failure_msg;
    failure_msg.task_id = task_name;
    failure_msg.status = "REJECTED";
    failure_msg.reason =
      "Task rejected: " + task_name + " request ignored because " + active_task_id + " is running.";
    task_response_publisher_->publish(failure_msg);
  }
}

void AutowareBridgeNode::publishTaskResponse(const std::string & task_id)
{
  if (autoware_bridge_util_->isTaskActive(task_id)) {
    TaskInfo task_status = autoware_bridge_util_->getTaskStatus(task_id);
    autoware_bridge_msgs::msg::TaskStatusResponse task_response =
      createTaskStatusResponse(task_id, task_status.status, task_status.reason);
    task_response_publisher_->publish(task_response);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Requested task_id: %s is not the active one.", task_id.c_str());
  }
}

void AutowareBridgeNode::cancelTaskCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string requested_task_id = msg->data;
  if (autoware_bridge_util_->isTaskActive(requested_task_id)) {
    std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->getActiveTaskPointer();
    if (active_task) {
      active_task->cancelRequested();
      publishCancelResponse(requested_task_id);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "task_id: %s cancel request failed as active task pointer is null.",
        requested_task_id.c_str());
    }
  } else {
    autoware_bridge_msgs::msg::TaskStatusResponse cancel_response;
    cancel_response.task_id = requested_task_id;
    cancel_response.status = "REJECTED";
    cancel_response.reason = "Requested task ID is not the active task.";
    cancel_response_publisher_->publish(cancel_response);
  }
}

void AutowareBridgeNode::publishCancelResponse(const std::string & task_id)
{
  if (autoware_bridge_util_->isTaskActive(task_id)) {
    TaskCancellationInfo task_cancellation_status =
      autoware_bridge_util_->getTaskStatus(task_id).cancel_info;
    autoware_bridge_msgs::msg::TaskStatusResponse cancel_response = createTaskStatusResponse(
      task_id, task_cancellation_status.status, task_cancellation_status.reason);
    cancel_response_publisher_->publish(cancel_response);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Requested task_id: %s is not the active one to cancel.",
      task_id.c_str());
  }
}

autoware_bridge_msgs::msg::TaskStatusResponse AutowareBridgeNode::createTaskStatusResponse(
  const std::string & task_id, const std::string & status, const std::string & reason)
{
  autoware_bridge_msgs::msg::TaskStatusResponse response;
  response.task_id = task_id;
  response.status = status;
  if (response.status != "SUCCESS") {
    response.reason = reason;
  } else {
    response.reason = "";
  }
  return response;
}

void AutowareBridgeNode::handleStatusRequest(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  autoware_bridge_util_->handleStatusRequest(request, response);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node and utility instance
  auto node = std::make_shared<rclcpp::Node>("autoware_bridge_node");
  auto autoware_bridge_util = std::make_shared<AutowareBridgeUtil>();
  std::atomic<bool> is_task_running(false);

  // Create task instances with correct arguments
  auto localization_task =
    std::make_shared<Localization>(node, autoware_bridge_util, is_task_running);
  auto route_planning_task =
    std::make_shared<RoutePlanning>(node, autoware_bridge_util, is_task_running);
  auto autonomous_driving_task =
    std::make_shared<AutonomousDriving>(node, autoware_bridge_util, is_task_running);

  // Create AutowareBridgeNode
  auto bridge_node = std::make_shared<AutowareBridgeNode>(
    autoware_bridge_util, localization_task, route_planning_task, autonomous_driving_task);

  rclcpp::spin(bridge_node);
  rclcpp::shutdown();
  return 0;
}
