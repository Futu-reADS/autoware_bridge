#include "autoware_bridge/autoware_bridge_util.hpp"

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>
#include <mutex>

void AutowareBridgeUtil::updateTask(
  const std::string & task_id, TaskRequestType request_type, const std::string & value, int number)
{
  std::lock_guard<std::mutex> lock(task_mutex_);

  auto it = task_map_.find(task_id);
  if (it != task_map_.end()) {
    TaskInfo & task_info = it->second;
    switch (request_type) {
      case TaskRequestType::STATUS:
        task_info.status = value;
        break;
      case TaskRequestType::RETRIES:
        task_info.retry_number = number;
        break;
      case TaskRequestType::TOTAL_RETRIES:
        task_info.total_retries = number;
        break;
      case TaskRequestType::CANCEL_STATUS:
        task_info.cancel_status = value;
        break;
      default:
        RCLCPP_WARN(rclcpp::get_logger("autoware_bridge_util"), "Request type is not valid.");
        break;
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("autoware_bridge_util"),
      "Requested task_id: %s is not the active one to update.", task_id.c_str());
  }
}

void AutowareBridgeUtil::updateTaskId(const std::string & task_id)
{
  task_map_.clear();
  task_map_.emplace(task_id, TaskInfo(3));
  RCLCPP_INFO(rclcpp::get_logger("autoware_bridge_util"), "Active task set to: %s", task_id.c_str());
}

void AutowareBridgeUtil::updateTaskStatus(const std::string & task_id, const std::string & status)
{
  updateTask(task_id, TaskRequestType::STATUS, status);

  if (status == "FAILED" || status == "TIMEOUT") {
    updateTask(task_id, TaskRequestType::CANCEL_STATUS, "FAILED_DUE_TO_TASK_FAILURE");
  } else if (status == "SUCCESS") {
    updateTask(task_id, TaskRequestType::CANCEL_STATUS, "FAILED_DUE_TO_TASK_SUCCESS");
  } else if (status == "CANCELLED") {
    updateTask(task_id, TaskRequestType::CANCEL_STATUS, "CANCELLED");
  }
}

void AutowareBridgeUtil::updateTaskRetries(const std::string & task_id, int retryNumber)
{
  updateTask(task_id, TaskRequestType::RETRIES, "", retryNumber);
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

TaskInfo AutowareBridgeUtil::getTaskStatus(const std::string & task_id)
{
  TaskInfo data(3);
  std::lock_guard<std::mutex> lock(task_mutex_);  // Ensure thread safety
  auto it = task_map_.find(task_id);


  if (it != task_map_.end()) {
    data = it->second;
  } else {
    //RCLCPP_WARN(
      //rclcpp::get_logger("autoware_bridge_util"), "Requested task_id: %s is not the active one.",
      //task_id.c_str());
      static rclcpp::Logger logger = rclcpp::get_logger("autoware_bridge_util");
      static auto clock = std::make_shared<rclcpp::Clock>();

    RCLCPP_INFO_THROTTLE(
      logger, *clock, 2000,
      "Requested task_id: %s is not the active one.",
      task_id.c_str());  
    }
  return data;
}

void AutowareBridgeUtil::setActiveTaskPtr(std::shared_ptr<BaseTask> task_ptr)
{
  active_task_ = task_ptr;
}

void AutowareBridgeUtil::clearActiveTaskPtr()
{
  active_task_ = nullptr;
}

std::shared_ptr<BaseTask> AutowareBridgeUtil::getActiveTaskPtr()
{
  return active_task_;
}

// use when status is requested in terms of service.
void AutowareBridgeUtil::handleStatusRequestSrvc(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("autoware_bridge_util"),
    "[handleStatusRequestSrvc] Received request for task_id: %s", request->task_id.c_str());

  if (isTaskActive(request->task_id)) {
    RCLCPP_INFO(
      rclcpp::get_logger("autoware_bridge_util"),
      "[handleStatusRequestSrvc] Task %s is active. Retrieving status.", request->task_id.c_str());

    // std::lock_guard<std::mutex> lock(task_mutex_);

    TaskInfo task_info = getTaskStatus(request->task_id);  // Retrieve task info

    RCLCPP_INFO(
      rclcpp::get_logger("autoware_bridge_util"),
      "[handleStatusRequestSrvc] Task status: %s, Retry number: %d, Total retries: %d",
      task_info.status.c_str(), task_info.retry_number, task_info.total_retries);

    response->status = task_info.status;
    response->retry_number = task_info.retry_number;
    response->total_retries = task_info.total_retries;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("autoware_bridge_util"),
      "[handleStatusRequestSrvc] Task %s is not active. Rejecting request.",
      request->task_id.c_str());

    response->status = "REJECTED";
    response->retry_number = 0;
    response->total_retries = 0;

    RCLCPP_INFO(
      rclcpp::get_logger("autoware_bridge_util"),
      "[handleStatusRequestSrvc] Response: status=REJECTED, retry_number=%d, "
      "total_retries=%d",
      response->retry_number, response->total_retries);
  }
}