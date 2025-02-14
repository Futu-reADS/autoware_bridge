#include "autoware_bridge/autoware_bridge_util.hpp"

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>  // Add this include for shared_ptr
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

void AutowareBridgeUtil::cancel_task(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  task_status_.erase(task_id);
}

void AutowareBridgeUtil::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Response> response)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (task_status_.find(request->task_id) != task_status_.end()) {
    response->status = task_status_[request->task_id];
  } else {
    response->status = "NOT_FOUND";
  }
}

void AutowareBridgeUtil::handle_cancel_request(
  const std::shared_ptr<autoware_bridge::srv::CancelTask_Request> request,
  std::shared_ptr<autoware_bridge::srv::CancelTask_Response> response)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (task_status_.find(request->task_id) != task_status_.end()) {
    task_status_.erase(request->task_id);
    response->success = true;
  } else {
    response->success = false;
  }
}
