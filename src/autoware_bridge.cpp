#include "autoware_bridge/autoware_bridge.hpp"

AutowareBridgeNode::AutowareBridgeNode()
: Node("autoware_bridge_node"),
  autoware_bridge_util_(),
  localization_task_(this->shared_from_this(), autoware_bridge_util_, is_task_running_),
  set_goal_task_(this->shared_from_this(), autoware_bridge_util_, is_task_running_),
  driving_task_(this->shared_from_this(), autoware_bridge_util_, is_task_running_),
  is_task_running_(false)  // Initialize task flag
{
  // Subscriptions handling
  subscription_1_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic1", 10,
    std::bind(&AutowareBridgeNode::topic_callback_1, this, std::placeholders::_1));

  subscription_2_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic2", 10,
    std::bind(&AutowareBridgeNode::topic_callback_2, this, std::placeholders::_1));

  subscription_3_ = this->create_subscription<std_msgs::msg::String>(  // Added third subscription
    "ui_to_autoware_topic3", 10,
    std::bind(&AutowareBridgeNode::topic_callback_3, this, std::placeholders::_1));

  // Publishers handling for task rejection status
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
}

void AutowareBridgeNode::topic_callback_1(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  if (is_task_running_.exchange(true)) {
    // Notify UI about failure
    publish_task_rejection_status("localization");
    return;
  }
  // this task_id will be provided by UI_bridge as parameter
  std::string task_id = autoware_bridge_util_.generate_task_id("localization");
  autoware_bridge_util_.update_task_status(task_id, "RUNNING");

  std::thread([this, task_id]() {
    localization_task_.execute(task_id);
    is_task_running_ = false;  // Mark task as completed
  }).detach();
}

void AutowareBridgeNode::topic_callback_2(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  if (is_task_running_.exchange(true)) {
    publish_task_rejection_status("set_goal");
    return;
  }

  std::string task_id = autoware_bridge_util_.generate_task_id("set_goal");
  autoware_bridge_util_.update_task_status(task_id, "PENDING");

  std::thread([this, task_id]() {
    set_goal_task_.execute(task_id);
    is_task_running_ = false;
  }).detach();
}

void AutowareBridgeNode::topic_callback_3(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  if (is_task_running_.exchange(true)) {
    publish_task_rejection_status("driving");
    return;
  }

  std::string task_id = autoware_bridge_util_.generate_task_id("driving");
  autoware_bridge_util_.update_task_status(task_id, "RUNNING");

  std::thread([this, task_id]() {
    driving_task_.execute(task_id);
    is_task_running_ = false;
  }).detach();
}

void AutowareBridgeNode::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  autoware_bridge_util_.handle_status_request(request, response);
}

void AutowareBridgeNode::handle_cancel_request(
  const std::shared_ptr<autoware_bridge::srv::CancelTask::Request> request,
  std::shared_ptr<autoware_bridge::srv::CancelTask::Response> response)
{
  std::string active_task = autoware_bridge_util_.get_active_task();  // Get the running task ID

  if (active_task == "NO_ACTIVE_TASK") {
    response->success = false;
    response->message = "No active task to cancel.";
    return;
  }

  if (request->task_id != active_task) {
    response->success = false;
    response->message = "Requested task ID does not match the currently running task.";
    return;
  }

  // Request cancellation for the correct task
  if (active_task.find("localization") != std::string::npos) {
    localization_task_.request_cancel();
  } else if (active_task.find("set_goal") != std::string::npos) {
    set_goal_task_.request_cancel();
  } else if (active_task.find("driving") != std::string::npos) {
    driving_task_.request_cancel();
  }

  response->success = true;
  response->message = "Task cancellation requested. The task will stop shortly.";
}

void AutowareBridgeNode::publish_task_rejection_status(const std::string & task_name)
{
  std::string active_task = autoware_bridge_util_.get_active_task();  // Get currently running task

  RCLCPP_WARN(
    this->get_logger(), "Task is already running (%s). Ignoring %s request.", active_task.c_str(),
    task_name.c_str());

  auto failure_msg = std_msgs::msg::String();
  failure_msg.data = "Task rejected: " + task_name + " request is ignored because " + active_task +
                     " is already running.";
  task_rejection_status_publisher_->publish(failure_msg);
}

// **Main Function**
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutowareBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
