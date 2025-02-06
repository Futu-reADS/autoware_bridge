#include "autoware_bridge/autoware_bridge.hpp"

#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

AutowareBridgeNode::AutowareBridgeNode() : Node("autoware_bridge_node")
{
  // Subscriptions
  subscription_1_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic1", 10,
    std::bind(&AutowareBridgeNode::topic_callback_1, this, std::placeholders::_1));

  subscription_2_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware_topic2", 10,
    std::bind(&AutowareBridgeNode::topic_callback_2, this, std::placeholders::_1));

  // Service
  // A service named "check_task_status" that will handle requests using the handle_status_request
  // function
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
    "check_task_status", std::bind(
                           &AutowareBridgeNode::handle_status_request, this, std::placeholders::_1,
                           std::placeholders::_2));

  // Start Task Executor Thread
  // A separate thread that runs the task_executor function, which is responsible for executing
  // tasks.
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

// Generate Unique Task ID
// This function generates a unique task ID by concatenating a
// given prefix with the current system time in nanoseconds.
// The result is a string that is likely to be unique for each task.
std::string AutowareBridgeNode::generate_task_id(const std::string & prefix)
{
  std::stringstream ss;
  ss << prefix << "_" << std::chrono::system_clock::now().time_since_epoch().count();
  return ss.str();
}

// Topic Callback for Localization
void AutowareBridgeNode::topic_callback_1(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  std::string task_id = generate_task_id("localization");

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_status_[task_id] = "PENDING";
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.emplace(task_id, [this, task_id]() { localization(task_id); });
  }
  queue_cv_.notify_one();
}

// Topic Callback for Set Goal
void AutowareBridgeNode::topic_callback_2(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  std::string task_id = generate_task_id("set_goal");

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_status_[task_id] = "PENDING";
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.emplace(task_id, [this, task_id]() { set_goal(task_id); });
  }
  queue_cv_.notify_one();
}

// Localization Task
void AutowareBridgeNode::localization(const std::string & task_id)
{
  update_task_status(task_id, "RUNNING");

  try {
    std::this_thread::sleep_for(2s);  // Simulated processing

    if (rand() % 5 == 0) {
      throw std::runtime_error("Simulated localization error");
    }

    update_task_status(task_id, "SUCCESS");
  } catch (const std::exception & e) {
    update_task_status(task_id, "ERROR");
  }
}

// Set Goal Task
void AutowareBridgeNode::set_goal(const std::string & task_id)
{
  update_task_status(task_id, "RUNNING");

  try {
    std::this_thread::sleep_for(3s);  // Simulated processing

    if (rand() % 4 == 0) {
      throw std::runtime_error("Simulated set goal error");
    }

    update_task_status(task_id, "SUCCESS");
  } catch (const std::exception & e) {
    update_task_status(task_id, "ERROR");
  }
}

// Executor Thread: Processes Tasks from Queue
void AutowareBridgeNode::task_executor()
{
  while (true) {
    std::pair<std::string, std::function<void()>> task;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() { return !task_queue_.empty() || stop_executor_; });

      if (stop_executor_ && task_queue_.empty()) return;

      task = std::move(task_queue_.front());
      task_queue_.pop();
    }

    // Execute Task
    task.second();
  }
}

// Update Task Status
void AutowareBridgeNode::update_task_status(const std::string & task_id, const std::string & status)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  task_status_[task_id] = status;
}

// Service to Get Task Status
void AutowareBridgeNode::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  auto it = task_status_.find(request->task_id);

  if (it != task_status_.end()) {
    response->status = it->second;
    response->success = true;
  } else {
    response->status = "NOT_FOUND";
    response->success = false;
  }
}

// Main Function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
