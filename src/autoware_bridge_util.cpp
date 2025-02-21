#include "autoware_bridge/autoware_bridge_util.hpp"

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>  // For shared_ptr
#include <mutex>

void AutowareBridgeUtil::update_task_status(
  const std::string & task_id, const std::string & status, const std::string & reason)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  auto it = task_status_.find(task_id);
  if (it == task_status_.end()) {
    // Create a new TaskInfo for this task id.
    TaskInfo info;
    info.task_id = task_id;
    info.status = status;
    info.reason = reason;
    // The retry numbers and service_response_status are initialized to default values.
    info.retry_number = 0;
    info.total_retries = 0;
    info.service_response_status = (status != "REJECTED");  // false only for rejected tasks
    // cancel_info remains default-initialized.
    if (status == "CANCELLED") {
      info.cancel_info.status = "SUCCESS";  // Assuming cancellation is successful
      info.cancel_info.reason = reason;
    }
    task_status_[task_id] = info;
  } else {
    // Update the existing TaskInfo entry.
    it->second.status = status;
    it->second.reason = reason;
    it->second.service_response_status = (status != "REJECTED");
    // If task is cancelled, update cancel_info
    if (status == "CANCELLED") {
      it->second.cancel_info.status = "SUCCESS";  // Assuming cancellation is successful
      it->second.cancel_info.reason = reason;
    }
  }
}

TaskInfo AutowareBridgeUtil::get_task_status(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(task_mutex_);  // Ensure thread safety
  auto it = task_status_.find(task_id);

  if (it != task_status_.end()) {
    return it->second;  // Return the task status if found
  }

  // Return a default TaskInfo with a "NOT_RUNNING" status
  TaskInfo task_info;
  task_info.task_id = task_id;                             // Assign the task_id
  task_info.status = "NOT_RUNNING";                        // Indicate it's not running
  task_info.reason = "Task not found in the known tasks";  // Optional
  // Ensure cancel_info is set to default values
  task_info.cancel_info.status = "UNKNOWN";
  task_info.cancel_info.reason = "No cancellation record available";

  return task_info;
}

// Active task tracking functions.
void AutowareBridgeUtil::set_active_task(
  const std::string & task_id, std::shared_ptr<BaseTask> task_ptr)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  // task_status_.clear();  // Clears all previous tasks
  // Mark the parameter as unused if needed:
  (void)task_id;
  active_task_ = task_ptr;
}

void AutowareBridgeUtil::clear_active_task()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  active_task_ = nullptr;  // Reset active task pointer
}

std::string AutowareBridgeUtil::get_active_task_id()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  for (const auto & task : task_status_) {
    if (task.second.status == "RUNNING") {
      return task.first;  // Return the active task's ID
    }
  }
  return "NO_ACTIVE_TASK";
}

std::shared_ptr<BaseTask> AutowareBridgeUtil::get_active_task_ptr()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  return active_task_;
}

// use when status is requested in terms of service.
void AutowareBridgeUtil::handle_status_request(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  std::lock_guard<std::mutex> lock(task_mutex_);

  auto it = task_status_.find(request->task_id);
  if (it != task_status_.end()) {
    const TaskInfo & task_info = it->second;

    response->success = true;
    response->message = task_info.status;
    response->retry_number = task_info.retry_number;
    response->total_retries = task_info.total_retries;
    response->rejection_reason = task_info.reason;

  } else if (request->task_id != get_active_task_id()) {
    response->success = false;
    response->message = "REJECTED";
    response->rejection_reason = "Requested task_id is not the last active one";
  } else {
    response->success = false;
    response->message = "NOT_RUNNING";
  }
}