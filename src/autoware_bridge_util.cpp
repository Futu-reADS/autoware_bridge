#include "autoware_bridge/autoware_bridge_util.hpp"

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>  // For shared_ptr
#include <mutex>


void AutowareBridgeUtil::updateTaskStatus(
  const std::string & task_id, const std::string & request_type, const std::string & value, int number)
{
  std::lock_guard<std::mutex> lock(task_mutex_);

  if (task_status_.empty()){
    task_status_[task_id] = TaskInfo();
  }

  auto it = task_status_.find(task_id);
  
  if ( it != task_status_.end() ){
    TaskInfo & task_info = it->second;
    switch(request_type){
      case "STATUS":
      task_info.status = value;
        break;
      case "REASON":
        it->second.reason = value;
        break;
      case "RETRIES":
        task_info.retry_number = number;
        break;
      case "TOTAL_RETRIES":
        task_info.total_retries = number;
        break;
      case "CANCEL_STATUS":
        task_info.cancel_info.status = value;
        break;
      case "CANCEL_REASON":
        task_info.cancel_info.reason = value;
        break;
      default:
        RCLCPP_WARN( this->get_logger(), "Request type is not valid: %s", request_type.c_str());
        break;
    }
  }
  else{
    RCLCPP_WARN( this->get_logger(), "Requested task_id: %s is not the active one to update.", 
    requested_task_id.c_str()); 
  }
}

bool AutowareBridgeUtil::isTaskActive(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  auto it = task_status_.find(task_id);
  if (it != task_status_.end()) {
    return true;
  }
  return false;
}

std::string AutowareBridgeUtil::getActiveTaskId()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  for (auto &[task_id, task_info] : task_status_) {
    return task_id;
  }
  return "NO_ACTIVE_TASK";
}

bool AutowareBridgeUtil::isActiveTaskIdEmpty()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  return task_status_.empty();
}

TaskInfo AutowareBridgeUtil::getTaskStatus(const std::string & task_id)
{
  TaskInfo data;
  std::lock_guard<std::mutex> lock(task_mutex_);  // Ensure thread safety
  auto it = task_status_.find(task_id);

  if (it != task_status_.end()) {
    data = it->second;
  }
  else{
    RCLCPP_WARN( this->get_logger(), "Requested task_id: %s is not the active one.", task_id.c_str()); 
  }
  return data
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