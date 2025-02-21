#include "autoware_bridge/autoware_bridge.hpp"

#include <functional>
#include <thread>

AutowareBridgeNode::AutowareBridgeNode(
  std::shared_ptr<AutowareBridgeUtil> util, std::shared_ptr<LocalizationTask> localization_task,
  std::shared_ptr<SetGoalTask> set_goal_task, std::shared_ptr<DrivingTask> driving_task)
: Node("autoware_bridge_node"),
  autoware_bridge_util_(util),
  localization_task_(localization_task),
  set_goal_task_(set_goal_task),
  driving_task_(driving_task),
  is_task_running_(false)
{
  // Subscriptions handling
  localization_request_subscription_ =
    this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(
      "/ftd_master/localization_request", 10,
      std::bind(&AutowareBridgeNode::localization_request_callback, this, std::placeholders::_1));

  // Subscribe to route planning requests
  route_planning_request_subscription_ =
    this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(
      "/ftd_master/route_planning_request", 10,
      std::bind(&AutowareBridgeNode::route_planning_request_callback, this, std::placeholders::_1));

  // Subscribe to autonomous driving requests (using std_msgs/String)
  autonomous_driving_request_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/ftd_master/autonomous_driving_request", 10,
    std::bind(
      &AutowareBridgeNode::autonomous_driving_request_callback, this, std::placeholders::_1));

  // Subscribe to cancellation requests (from UI/ftd_master)
  cancel_task_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/ftd_master/cancel_task", 10,
    std::bind(&AutowareBridgeNode::cancel_task_callback, this, std::placeholders::_1));

  // Publisher for task execution responses (topic-based)
  task_response_publisher_ = this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
    "/autoware_bridge/task_response", 10);

  // Publisher for cancellation responses
  cancel_response_publisher_ =
    this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
      "/autoware_bridge/cancel_task_response", 10);

  // Publisher for task rejection reason
  task_rejection_reason_publisher_ =
    this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
      "/autoware_bridge/rejection_reason", 10);

  // Services handling
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
    "check_task_status", std::bind(
                           &AutowareBridgeNode::handle_status_request, this, std::placeholders::_1,
                           std::placeholders::_2));
}

AutowareBridgeNode::~AutowareBridgeNode()
{
  // Optionally: join or signal threads if you decide to track them instead of detaching.
}

// Task Handling Callbacks

void AutowareBridgeNode::localization_request_callback(
  const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{
  if (is_task_running_.exchange(true)) {
    publish_task_rejection_reason("localization");
    RCLCPP_WARN(this->get_logger(), "A task is already running. Localization request rejected.");
    return;
  }
  // Use the task_id provided in the message
  std::string requested_task_id = msg->task_id.data;
  geometry_msgs::msg::PoseStamped pose = msg->pose;
  RCLCPP_INFO(
    this->get_logger(), "Received localization request with task_id: %s",
    requested_task_id.c_str());
  // Update status and set active task using the provided task id
  autoware_bridge_util_->update_task_status(requested_task_id, "PENDING");
  autoware_bridge_util_->set_active_task(requested_task_id, localization_task_);

  std::thread([this, requested_task_id, pose]() {
    std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->get_active_task_ptr();
    if (active_task) {
      active_task->execute(requested_task_id, pose);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Active task pointer is null for task_id: %s",
        requested_task_id.c_str());
    }
    is_task_running_ = false;
    // Publish final execution response:
    publishTaskResponse(requested_task_id);
    autoware_bridge_util_->clear_active_task();
  }).detach();
}

void AutowareBridgeNode::route_planning_request_callback(
  const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{
  if (is_task_running_.exchange(true)) {
    RCLCPP_WARN(this->get_logger(), "A task is already running. Route planning request rejected.");
    publish_task_rejection_reason("route_planning");
    return;
  }
  // Use the task_id provided by the message
  std::string task_id = msg->task_id.data;
  geometry_msgs::msg::PoseStamped pose = msg->pose;
  RCLCPP_INFO(
    this->get_logger(), "Received route planning request with task_id: %s", task_id.c_str());

  autoware_bridge_util_->update_task_status(task_id, "PENDING");
  autoware_bridge_util_->set_active_task(task_id, set_goal_task_);
  std::thread([this, task_id, pose]() {
    std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->get_active_task_ptr();
    if (active_task) {
      active_task->execute(task_id, pose);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Active task pointer is null for task_id: %s", task_id.c_str());
    }
    is_task_running_ = false;
    // Publish final execution response:
    publishTaskResponse(task_id);
    autoware_bridge_util_->clear_active_task();
  }).detach();
}
void AutowareBridgeNode::autonomous_driving_request_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (is_task_running_.exchange(true)) {
    RCLCPP_WARN(
      this->get_logger(), "A task is already running. Autonomous driving request rejected.");
    publish_task_rejection_reason("autonomous_driving");
    return;
  }
  // For autonomous driving, the message itself is the task_id.
  std::string task_id = msg->data;
  geometry_msgs::msg::PoseStamped dummy_pose;
  RCLCPP_INFO(
    this->get_logger(), "Received autonomous driving request with task_id: %s", task_id.c_str());

  autoware_bridge_util_->update_task_status(task_id, "PENDING");
  autoware_bridge_util_->set_active_task(task_id, driving_task_);
  std::thread([this, task_id, dummy_pose]() {
    std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->get_active_task_ptr();
    if (active_task) {
      active_task->execute(task_id, dummy_pose);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Active task pointer is null for task_id: %s", task_id.c_str());
    }
    is_task_running_ = false;
    // Publish final execution response:
    publishTaskResponse(task_id);
    autoware_bridge_util_->clear_active_task();
  }).detach();
}

void AutowareBridgeNode::publishTaskResponse(const std::string & task_id)
{
  auto task_response = createTaskStatusResponse(task_id);
  task_response_publisher_->publish(task_response);
}

void AutowareBridgeNode::publishCancelResponse(const std::string & task_id)
{
  auto cancel_response = createTaskStatusResponse(task_id);
  cancel_response_publisher_->publish(cancel_response);
}
autoware_bridge_msgs::msg::TaskStatusResponse AutowareBridgeNode::createTaskStatusResponse(
  const std::string & task_id)
{
  autoware_bridge_msgs::msg::TaskStatusResponse task_response;
  TaskInfo data = autoware_bridge_util_->get_task_status(task_id);
  task_response.task_id = data.task_id;
  task_response.status = data.status;
  if (task_response.status != "SUCCESS") {
    task_response.reason = data.reason;
  } else {
    task_response.reason = "";
  }
  return task_response;
}
// Service Handlers
void AutowareBridgeNode::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  autoware_bridge_util_->handle_status_request(request, response);
}

void AutowareBridgeNode::cancel_task_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string requested_task_id = msg->data;
  std::string active_task_id = autoware_bridge_util_->get_active_task_id();
  std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->get_active_task_ptr();

  autoware_bridge_msgs::msg::TaskStatusResponse cancel_response;
  cancel_response.task_id = requested_task_id;

  if (active_task_id == "NO_ACTIVE_TASK") {
    RCLCPP_WARN(this->get_logger(), "UI requested cancellation, but no task is currently running.");
    cancel_response.status = "REJECTED";
    cancel_response.reason = "No active task is currently running.";
    cancel_response_publisher_->publish(cancel_response);
    return;
  }

  if (requested_task_id != active_task_id) {
    RCLCPP_WARN(
      this->get_logger(),
      "UI requested cancellation for task [%s], but currently running task is [%s]. Ignoring "
      "request.",
      requested_task_id.c_str(), active_task_id.c_str());
    cancel_response.status = "REJECTED";
    cancel_response.reason = "Requested task ID does not match the active task.";
    cancel_response_publisher_->publish(cancel_response);
    return;
  }

  if (active_task) {
    active_task->request_cancel();
    // Publish cancellation response:
    publishCancelResponse(requested_task_id);
    RCLCPP_INFO(
      this->get_logger(), "UI cancel request: Task [%s] cancellation requested.",
      active_task_id.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "UI cancel request: Active task pointer is null.");
  }
}

// Utility to publish task rejection messages
void AutowareBridgeNode::publish_task_rejection_reason(const std::string & task_id)
{
  std::string active_task_id = autoware_bridge_util_->get_active_task_id();

  RCLCPP_WARN(
    this->get_logger(), "Task is already running (%s). Ignoring %s request.",
    active_task_id.c_str(), task_id.c_str());

  autoware_bridge_msgs::msg::TaskStatusResponse failure_msg;
  failure_msg.task_id = task_id;
  failure_msg.status = "REJECTED";
  failure_msg.reason =
    "Task rejected: " + task_id + " request ignored because " + active_task_id + " is running.";
  task_rejection_reason_publisher_->publish(failure_msg);
}

// **Main Function**
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node and utility instance
  auto node = std::make_shared<rclcpp::Node>("autoware_bridge_node");
  auto autoware_bridge_util = std::make_shared<AutowareBridgeUtil>();
  std::atomic<bool> is_task_running(false);

  // Create task instances with correct arguments
  auto localization_task =
    std::make_shared<LocalizationTask>(node, autoware_bridge_util, is_task_running);
  auto set_goal_task = std::make_shared<SetGoalTask>(node, autoware_bridge_util, is_task_running);
  auto driving_task = std::make_shared<DrivingTask>(node, autoware_bridge_util, is_task_running);

  // Create AutowareBridgeNode
  auto bridge_node = std::make_shared<AutowareBridgeNode>(
    autoware_bridge_util, localization_task, set_goal_task, driving_task);

  rclcpp::spin(bridge_node);
  rclcpp::shutdown();
  return 0;
}
