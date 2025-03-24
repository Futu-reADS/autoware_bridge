#include "autoware_bridge/autoware_bridge_util.hpp"
#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>
#include <mutex>

void AutowareBridgeUtil::updateTaskStatus(
  const std::string & task_id, TaskRequestType request_type, const std::string & value, int number)
{
  std::lock_guard<std::mutex> lock(task_mutex_);

  // If there is already a task and it's not the same as the new task_id, clear the map.
  if (!task_map_.empty() && task_map_.find(task_id) == task_map_.end()) {
    task_map_.clear();
  }

  // Ensure that the task entry exists.
  if (task_map_.find(task_id) == task_map_.end()) {
    task_map_[task_id] = TaskInfo();
  }

  /* if (task_map_.empty()) {
    task_map_[task_id] = TaskInfo();
  } */

  auto it = task_map_.find(task_id);

  if (it != task_map_.end()) {
    TaskInfo & task_info = it->second;
    switch (request_type) {
      case TaskRequestType::STATUS:
        task_info.status = value;
        break;
      case TaskRequestType::REASON:
        task_info.reason = value;
        break;
      case TaskRequestType::RETRIES:
        task_info.retry_number = number;
        break;
      case TaskRequestType::TOTAL_RETRIES:
        task_info.total_retries = number;
        break;
      case TaskRequestType::CANCEL_STATUS:
        task_info.cancel_info.status = value;
        break;
      case TaskRequestType::CANCEL_REASON:
        task_info.cancel_info.reason = value;
        break;
      default:
        RCLCPP_WARN(rclcpp::get_logger("autoware_bridge_util"), "Request type is not valid.");
        break;
    }
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("autoware_bridge_util"),
      "Requested task_id: %s is not the active one to update.", task_id.c_str());
  }
}

void AutowareBridgeUtil::updateFailStatus(const std::string & task_id, const std::string & reason)
{
  updateTaskStatus(task_id, TaskRequestType::STATUS, "FAILED");
  updateTaskStatus(task_id, TaskRequestType::REASON, reason);
}

void AutowareBridgeUtil::updateSuccessStatus(const std::string & task_id)
{
  updateTaskStatus(task_id, TaskRequestType::STATUS, "SUCCESS");
}

void AutowareBridgeUtil::updateCancellationStatus(
  const std::string & task_id, const std::string & reason)
{
  updateTaskStatus(task_id, TaskRequestType::STATUS, "CANCELLED");
  updateTaskStatus(task_id, TaskRequestType::REASON, reason);

  // Update cancel-specific fields [cancel_info]
  updateTaskStatus(task_id, TaskRequestType::CANCEL_STATUS, "CANCELLED");
  updateTaskStatus(task_id, TaskRequestType::CANCEL_REASON, reason);
}

void AutowareBridgeUtil::updateRunningStatusWithRetries(const std::string & task_id, const int total_retries)
{
  // Update task status to RUNNING
  updateTaskStatus(task_id, TaskRequestType::STATUS, "RUNNING");
  // Update total retries
  updateTaskStatus(task_id, TaskRequestType::TOTAL_RETRIES, "", total_retries);
}

void AutowareBridgeUtil::updateHaltStatus(const std::string & task_id, const std::string & reason)
{
  updateTaskStatus(task_id, TaskRequestType::STATUS, "HALTED");
  updateTaskStatus(task_id, TaskRequestType::REASON, reason);
}

bool AutowareBridgeUtil::isTaskActive(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  auto it = task_map_.find(task_id);
  if (it != task_map_.end()) {
    return true;
  }
  return false;
}

std::string AutowareBridgeUtil::getActiveTaskId()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  for (auto & [task_id, task_info] : task_map_) {
    return task_id;
  }
  return "NO_ACTIVE_TASK";
}

bool AutowareBridgeUtil::isActiveTaskIdEmpty()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  return task_map_.empty();
}

TaskInfo AutowareBridgeUtil::getTaskStatus(const std::string & task_id)
{
  TaskInfo data;
  std::lock_guard<std::mutex> lock(task_mutex_);  // Ensure thread safety
  auto it = task_map_.find(task_id);

  if (it != task_map_.end()) {
    data = it->second;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("autoware_bridge_util"), "Requested task_id: %s is not the active one.",
      task_id.c_str());
  }
  return data;
}

void AutowareBridgeUtil::setActiveTask(std::shared_ptr<BaseTask> task_ptr)
{
  // std::lock_guard<std::mutex> lock(task_mutex_);
  active_task_ = task_ptr;
}

void AutowareBridgeUtil::clearActiveTask()
{
  // std::lock_guard<std::mutex> lock(task_mutex_);
  active_task_ = nullptr;  // Reset active task pointer
}

std::shared_ptr<BaseTask> AutowareBridgeUtil::getActiveTaskPointer()
{
  // std::lock_guard<std::mutex> lock(task_mutex_);
  return active_task_;
}

// use when status is requested in terms of service.
void AutowareBridgeUtil::handleStatusRequest(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("autoware_bridge_util"), 
              "[handleStatusRequest] Received request for task_id: %s", 
              request->task_id.c_str());

  if (isTaskActive(request->task_id)) {
    RCLCPP_INFO(rclcpp::get_logger("autoware_bridge_util"), 
                "[handleStatusRequest] Task %s is active. Retrieving status.", 
                request->task_id.c_str());

    //std::lock_guard<std::mutex> lock(task_mutex_);
    
    TaskInfo task_info = getTaskStatus(request->task_id);  // Retrieve task info

    RCLCPP_INFO(rclcpp::get_logger("autoware_bridge_util"),
                "[handleStatusRequest] Task status: %s, Retry number: %d, Total retries: %d, Reason: %s",
                task_info.status.c_str(), 
                task_info.retry_number, 
                task_info.total_retries, 
                task_info.reason.c_str());

    response->status = task_info.status;
    response->retry_number = task_info.retry_number;
    response->total_retries = task_info.total_retries;
    response->reason = task_info.reason;
    
  } else {
    RCLCPP_WARN(rclcpp::get_logger("autoware_bridge_util"),
                "[handleStatusRequest] Task %s is not active. Rejecting request.", 
                request->task_id.c_str());

    response->status = "REJECTED";
    response->reason = "Requested task_id is not the last active one";
    response->retry_number = 0;
    response->total_retries = 0;

    RCLCPP_INFO(rclcpp::get_logger("autoware_bridge_util"),
                "[handleStatusRequest] Response: status=REJECTED, reason=%s, retry_number=%d, total_retries=%d",
                response->reason.c_str(),
                response->retry_number,
                response->total_retries);
  }
}