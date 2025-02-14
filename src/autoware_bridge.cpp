#include "autoware_bridge/autoware_bridge.hpp"

AutowareBridgeNode::AutowareBridgeNode()
: Node("autoware_bridge_node"),
  autoware_bridge_util_(),  // Initialize instance
  localization_task_(this->shared_from_this(), autoware_bridge_util_),
  set_goal_task_(this->shared_from_this(), autoware_bridge_util_)
{
  // subscriptions handling
  subscription_1_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic1", 10,
    std::bind(&AutowareBridgeNode::topic_callback_1, this, std::placeholders::_1));

  subscription_2_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic2", 10,
    std::bind(&AutowareBridgeNode::topic_callback_2, this, std::placeholders::_1));

  // services handling
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
    "check_task_status", std::bind(
                           &AutowareBridgeNode::handle_status_request, this, std::placeholders::_1,
                           std::placeholders::_2));

  cancel_service_ = this->create_service<autoware_bridge::srv::CancelTask>(
    "cancel_task", std::bind(
                     &AutowareBridgeNode::handle_cancel_request, this, std::placeholders::_1,
                     std::placeholders::_2));

  executor_thread_ = std::thread(&AutowareBridgeNode::task_executor, this);
}

AutowareBridgeNode::~AutowareBridgeNode()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_executor_ = true;
  }
  queue_cv_.notify_one();
  executor_thread_.join();
}

void AutowareBridgeNode::topic_callback_1(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  std::string task_id = autoware_bridge_util_.generate_task_id("localization");
  autoware_bridge_util_.update_task_status(task_id, "PENDING");

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.emplace(task_id, [this, task_id]() { localization_task_.execute(task_id); });
  }
  queue_cv_.notify_one();
}

void AutowareBridgeNode::topic_callback_2(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  std::string task_id = autoware_bridge_util_.generate_task_id("set_goal");
  autoware_bridge_util_.update_task_status(task_id, "PENDING");

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.emplace(task_id, [this, task_id]() { set_goal_task_.execute(task_id); });
  }
  queue_cv_.notify_one();
}

void AutowareBridgeNode::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  autoware_bridge_util_.handle_status_request(request, response);
}
/*
void AutowareBridgeNode::handle_cancel_request(
  const std::shared_ptr<autoware_bridge::srv::CancelTask::Request> request,
  std::shared_ptr<autoware_bridge::srv::CancelTask::Response> response)
{
  // autoware_bridge_util_.handle_cancel_request(request, response);
} */

void AutowareBridgeNode::handle_cancel_request(
  const std::shared_ptr<autoware_bridge::srv::CancelTask::Request> /* request */,
  std::shared_ptr<autoware_bridge::srv::CancelTask::Response> /* response */)
{
  // Your actual cancellation logic here
}

void AutowareBridgeNode::task_executor()
{
  while (true) {
    std::pair<std::string, std::function<void()>> task;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() { return !task_queue_.empty() || stop_executor_; });

      if (stop_executor_ && task_queue_.empty()) {
        return;
      }

      task = std::move(task_queue_.front());
      task_queue_.pop();
    }

    // Execute Task
    task.second();
  }
}

// **Main Function**
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
