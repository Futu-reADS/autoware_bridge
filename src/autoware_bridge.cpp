#include "autoware_bridge/autoware_bridge.hpp"

#include <chrono>
#include <exception>  // For exception handling
#include <sstream>

using namespace std::chrono_literals;

AutowareBridgeNode::AutowareBridgeNode() : Node("autoware_bridge_node")
{
  // Subscription to UI_bridge topic
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "ui_to_autoware", 10,
    std::bind(&AutowareBridgeNode::topic_callback, this, std::placeholders::_1));

  // Service to check task status
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
    "check_task_status", std::bind(
                           &AutowareBridgeNode::handle_status_request, this, std::placeholders::_1,
                           std::placeholders::_2));

  // Register shutdown callback
  rclcpp::on_shutdown([this]() { graceful_shutdown(); });
}

std::string AutowareBridgeNode::generate_task_id()
{
  std::stringstream ss;
  ss << "task_" << std::chrono::system_clock::now().time_since_epoch().count();
  return ss.str();
}

void AutowareBridgeNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  std::string task_id = generate_task_id();

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_status_[task_id] = "PENDING";
  }

  std::thread task_thread([this, task_id]() {
    update_task_status(task_id, "RUNNING");

    try {
      // Simulated task processing
      std::this_thread::sleep_for(2s);

      // Simulate possible error
      if (rand() % 5 == 0) {  // 20% chance of failure
        throw std::runtime_error("Simulated task error");
      }

      update_task_status(task_id, "SUCCESS");
      RCLCPP_INFO(this->get_logger(), "Task %s completed with status: SUCCESS", task_id.c_str());

    } catch (const std::exception & e) {
      update_task_status(task_id, "ERROR");
      RCLCPP_ERROR(this->get_logger(), "Task %s failed with error: %s", task_id.c_str(), e.what());
    }
  });

  // Add thread to active threads list
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_threads_.emplace_back(std::move(task_thread));
  }
}

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

void AutowareBridgeNode::update_task_status(const std::string & task_id, const std::string & status)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  task_status_[task_id] = status;
}

void AutowareBridgeNode::graceful_shutdown()
{
  RCLCPP_INFO(
    this->get_logger(), "Shutting down gracefully. Waiting for active tasks to complete...");

  std::lock_guard<std::mutex> lock(task_mutex_);
  for (auto & thread : active_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  RCLCPP_INFO(this->get_logger(), "All active tasks completed. Node shutting down.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
