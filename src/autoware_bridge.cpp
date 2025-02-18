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
  subscription_1_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic1", 10,
    std::bind(&AutowareBridgeNode::topic_callback_1, this, std::placeholders::_1));

  subscription_2_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic2", 10,
    std::bind(&AutowareBridgeNode::topic_callback_2, this, std::placeholders::_1));

  subscription_3_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic3", 10,
    std::bind(&AutowareBridgeNode::topic_callback_3, this, std::placeholders::_1));

  // Publisher for task rejection status
  task_rejection_status_publisher_ =
    this->create_publisher<std_msgs::msg::String>("task_status", 10);

  // Services handling
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
    "check_task_status", std::bind(
                           &AutowareBridgeNode::handle_status_request, this, std::placeholders::_1,
                           std::placeholders::_2));

  cancel_service_ = this->create_service<autoware_bridge::srv::CancelTask>(
    "cancel_task", std::bind(
                     &AutowareBridgeNode::handle_cancel_request, this, std::placeholders::_1,
                     std::placeholders::_2));
}

AutowareBridgeNode::~AutowareBridgeNode()
{
  // Optionally: join or signal threads if you decide to track them instead of detaching.
}

// Task Handling Callbacks
void AutowareBridgeNode::topic_callback_1(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  if (is_task_running_.exchange(true)) {
    publish_task_rejection_status("localization");
    return;
  }

  std::string task_id = autoware_bridge_util_->generate_task_id("localization");
  autoware_bridge_util_->update_task_status(task_id, "PENDING");
  // Set the active task pointer and active task id
  autoware_bridge_util_->set_active_task(task_id, localization_task_);

  std::thread([this, task_id]() {
    localization_task_->execute(task_id);
    is_task_running_ = false;
    autoware_bridge_util_->clear_active_task();
  }).detach();
}

void AutowareBridgeNode::topic_callback_2(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  if (is_task_running_.exchange(true)) {
    publish_task_rejection_status("set_goal");
    return;
  }

  std::string task_id = autoware_bridge_util_->generate_task_id("set_goal");
  autoware_bridge_util_->update_task_status(task_id, "PENDING");
  autoware_bridge_util_->set_active_task(task_id, set_goal_task_);

  std::thread([this, task_id]() {
    set_goal_task_->execute(task_id);
    is_task_running_ = false;
    autoware_bridge_util_->clear_active_task();
  }).detach();
}

void AutowareBridgeNode::topic_callback_3(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  if (is_task_running_.exchange(true)) {
    publish_task_rejection_status("driving");
    return;
  }

  std::string task_id = autoware_bridge_util_->generate_task_id("driving");
  autoware_bridge_util_->update_task_status(task_id, "PENDING");
  autoware_bridge_util_->set_active_task(task_id, driving_task_);

  std::thread([this, task_id]() {
    driving_task_->execute(task_id);
    is_task_running_ = false;
    autoware_bridge_util_->clear_active_task();
  }).detach();
}

// Service Handlers
void AutowareBridgeNode::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  autoware_bridge_util_->handle_status_request(request, response);
}

void AutowareBridgeNode::handle_cancel_request(
  const std::shared_ptr<autoware_bridge::srv::CancelTask::Request> request,
  std::shared_ptr<autoware_bridge::srv::CancelTask::Response> response)
{
  std::string active_task_id = autoware_bridge_util_->get_active_task_id();
  std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->get_active_task_ptr();

  if (active_task_id == "NO_ACTIVE_TASK") {
    response->success = false;
    response->message = "No active task to cancel.";
    return;
  }

  if (request->task_id != active_task_id) {
    response->success = false;
    response->message = "Requested task ID does not match the currently running task.";
    return;
  }

    if (active_task) {
    active_task->request_cancel();
    response->success = true;
    response->message = "Task cancellation requested.";
  } else {
    response->success = false;
    response->message = "Active task pointer is null.";
  }
}

// Utility to publish task rejection messages
void AutowareBridgeNode::publish_task_rejection_status(const std::string & task_name)
{
  std::string active_task_id = autoware_bridge_util_->get_active_task_id();

  RCLCPP_WARN(
    this->get_logger(), "Task is already running (%s). Ignoring %s request.",
    active_task_id.c_str(), task_name.c_str());

  auto failure_msg = std_msgs::msg::String();
  failure_msg.data =
    "Task rejected: " + task_name + " request ignored because " + active_task_id + " is running.";
  task_rejection_status_publisher_->publish(failure_msg);
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
