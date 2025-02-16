#include "autoware_bridge/autoware_bridge_util.hpp"

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>  // For shared_ptr
#include <mutex>

std::string AutowareBridgeUtil::generate_task_id(const std::string & task_name)
{
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time now = clock.now();
  return task_name + "_" + std::to_string(now.nanoseconds());
}

void AutowareBridgeUtil::update_task_status(const std::string & task_id, const std::string & status)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  task_status_[task_id] = status;
}

std::string AutowareBridgeUtil::get_task_status(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(task_mutex_);  // Ensure thread safety

  auto it = task_status_.find(task_id);
  if (it != task_status_.end()) {
    return it->second;  // Return the task status if found
  }

  return "NOT_FOUND";  // Task ID does not exist
}

std::string AutowareBridgeUtil::get_active_task()
{
  std::lock_guard<std::mutex> lock(task_mutex_);  // Ensure thread safety

  for (const auto & task : task_status_) {
    if (task.second == "RUNNING") {
      return task.first;  // Return the first active task found
    }
  }

  return "NO_ACTIVE_TASK";  // No running task found
}

bool AutowareBridgeUtil::cancel_task(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  auto it = task_status_.find(task_id);
  if (it != task_status_.end()) {
    task_status_.erase(it);
    return true;  // Successfully canceled
  }
  return false;  // Task not found
}

void AutowareBridgeUtil::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Response> response)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  auto it = task_status_.find(request->task_id);
  response->status = (it != task_status_.end()) ? it->second : "NOT_FOUND";
}

void AutowareBridgeUtil::handle_cancel_request(
  const std::shared_ptr<autoware_bridge::srv::CancelTask_Request> request,
  std::shared_ptr<autoware_bridge::srv::CancelTask_Response> response)
{
  response->success = cancel_task(request->task_id);
}
