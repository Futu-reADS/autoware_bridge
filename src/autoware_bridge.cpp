#include "autoware_bridge/autoware_bridge.hpp"

#include <functional>
#include <thread>

AutowareBridgeNode::AutowareBridgeNode(
  std::shared_ptr<AutowareBridgeUtil> util, std::shared_ptr<LocalizationTask> localization_task,
  std::shared_ptr<SetGoalTask> set_goal_task, std::shared_ptr<DrivingTask> driving_task)
: Node("autoware_bridge_node"),
  autoware_bridge_util_(util),
  localization_task_(localization_task),
  set_goal_task_(set_goal_task),
  driving_task_(driving_task),
  is_task_running_(false)
{
  // Subscriptions handling
  localization_request_subscription_ =
    this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(
      "/ftd_master/localization_request", 10,
      std::bind(&AutowareBridgeNode::localization_request_callback, this, std::placeholders::_1));

  // Subscribe to route planning requests
  route_planning_request_subscription_ =
    this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(
      "/ftd_master/route_planning_request", 10,
      std::bind(&AutowareBridgeNode::route_planning_request_callback, this, std::placeholders::_1));

  // Subscribe to autonomous driving requests (using std_msgs/String)
  autonomous_driving_request_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/ftd_master/autonomous_driving_request", 10,
    std::bind(
      &AutowareBridgeNode::autonomous_driving_request_callback, this, std::placeholders::_1));

  // Subscribe to cancellation requests (from UI/ftd_master)
  cancel_task_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/ftd_master/cancel_task", 10,
    std::bind(&AutowareBridgeNode::cancel_task_callback, this, std::placeholders::_1));

  // Publisher for task execution responses (topic-based)
  task_response_publisher_ = this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
    "/autoware_bridge/task_response", 10);

  // Publisher for cancellation responses
  cancel_response_publisher_ =
    this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
      "/autoware_bridge/cancel_task_response", 10);

  // Publisher for task rejection reason
  task_rejection_reason_publisher_ =
    this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
      "/autoware_bridge/rejection_reason", 10);

  // Services handling
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
    "check_task_status", std::bind(
                           &AutowareBridgeNode::handle_status_request, this, std::placeholders::_1,
                           std::placeholders::_2));
}

AutowareBridgeNode::~AutowareBridgeNode()
{
  // Optionally: join or signal threads if you decide to track them instead of detaching.
}

// Task Handling Callbacks

void AutowareBridgeNode::localizationRequestCallback(
  const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{

  if (isTaskRejected("localization")) {
    return;
  }

  std::string requested_task_id = msg->task_id.data;
  geometry_msgs::msg::PoseStamped pose_stamped = msg->pose_stamped;

  RCLCPP_INFO( this->get_logger(), "Received localization request with task_id: %s",
    requested_task_id.c_str());

  autoware_bridge_util_->updateTaskStatus(requested_task_id, "STATUS", "PENDING");
  autoware_bridge_util_->setActiveTask(localization_task_);
  startThreadExecution(requested_task_id, pose_stamped);
}

void AutowareBridgeNode::routePlanningRequestCallback(
  const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{
  if (isTaskRejected("route_planning")) {
    return;
  }
  
  std::string requested_task_id = msg->task_id.data;
  geometry_msgs::msg::PoseStamped pose_stamped = msg->pose_stamped;

  RCLCPP_INFO( this->get_logger(), "Received localization request with task_id: %s",
    requested_task_id.c_str());

  autoware_bridge_util_->updateTaskStatus(requested_task_id, "STATUS", "PENDING");
  autoware_bridge_util_->setActiveTask(route_planning_task_);
  startThreadExecution(requested_task_id, pose_stamped);
}

void AutowareBridgeNode::autonomousDrivingRequestCallback(
  const std_msgs::msg::String::SharedPtr msg)
{

  if (isTaskRejected("autonomous_driving")) {
    return;
  }
  
  std::string requested_task_id = msg->task_id.data;
  geometry_msgs::msg::PoseStamped dummy_pose_stamped;

  RCLCPP_INFO( this->get_logger(), "Received localization request with task_id: %s",
    requested_task_id.c_str());

  autoware_bridge_util_->updateTaskStatus(requested_task_id, "STATUS", "PENDING");
  autoware_bridge_util_->setActiveTask(autonomous_driving_task_);
  startThreadExecution(requested_task_id, dummy_pose_stamped);
}

bool AutowareBridgeNode::isTaskRejected( const std::string & task_name)
{
  if (is_task_running_.exchange(true)) {
    RCLCPP_WARN(this->get_logger(), "A task is already running. %s request rejected.", task_name.c_str());
    publishTaskRejectionReason(task_name);
    return true;
  }
  return false;
}

void AutowareBridgeNode::startThreadExecution( const std::string & requested_task_id, const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->getActiveTaskPointer();
  if(active_task){
    std::thread([this, requested_task_id, pose_stamped]() {
      active_task->execute(requested_task_id, pose_stamped);
      autoware_bridge_util_->clear_active_task();
    }).detach();
  }
  else{
    RCLCPP_ERROR( this->get_logger(), "Active task pointer is null for task_id: %s",
      requested_task_id.c_str());
  }
}

void AutowareBridgeNode::publishTaskRejectionReason(const std::string & task_name)
{

  std::string active_task_id = autoware_bridge_util_->getActiveTaskId();
  if (active_task_id == "NO_ACTIVE_TASK") {
    RCLCPP_WARN(this->get_logger(), "No active task is currently running. Ignoring %s request.", task_name.c_str());
    return;
  }
  else{
    autoware_bridge_msgs::msg::TaskStatusResponse failure_msg;
    failure_msg.task_id = task_name;
    failure_msg.status = "REJECTED";
    failure_msg.reason = "Task rejected: " + task_name + " request ignored because " + active_task_id + " is running.";
    task_rejection_reason_publisher_->publish(failure_msg);
  }
}

void AutowareBridgeNode::publishTaskResponse(const std::string & task_id)
{
  auto task_response = createTaskStatusResponse(task_id);
  task_response_publisher_->publish(task_response);
}

void AutowareBridgeNode::publishCancelResponse(const std::string & task_id)
{
  auto cancel_response = createTaskStatusResponse(task_id);
  cancel_response_publisher_->publish(cancel_response);
}
autoware_bridge_msgs::msg::TaskStatusResponse AutowareBridgeNode::createTaskStatusResponse(
  const std::string & task_id)
{


  autoware_bridge_msgs::msg::TaskStatusResponse task_response;
  TaskInfo data;




  autoware_bridge_util_->get_task_status(task_id, &data);
  task_response.task_id = data.task_id;
  task_response.status = data.status;
  if (task_response.status != "SUCCESS") {
    task_response.reason = data.reason;
  } else {
    task_response.reason = "";
  }
  return task_response;
}




// Service Handlers
void AutowareBridgeNode::handleStatusRequest(
  const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
  std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  autoware_bridge_util_->handleStatusRequest(request, response);
}

void AutowareBridgeNode::cancelTaskCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string requested_task_id = msg->data;
  std::string active_task_id = autoware_bridge_util_->get_active_task_id();
  std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->get_active_task_ptr();

  autoware_bridge_msgs::msg::TaskStatusResponse cancel_response;
  cancel_response.task_id = requested_task_id;

  if (active_task_id == "NO_ACTIVE_TASK") {
    RCLCPP_WARN(this->get_logger(), "UI requested cancellation, but no task is currently running.");
    cancel_response.status = "REJECTED";
    cancel_response.reason = "No active task is currently running.";
    cancel_response_publisher_->publish(cancel_response);
    return;
  }

  if (requested_task_id != active_task_id) {
    RCLCPP_WARN(
      this->get_logger(),
      "UI requested cancellation for task [%s], but currently running task is [%s]. Ignoring "
      "request.",
      requested_task_id.c_str(), active_task_id.c_str());
    cancel_response.status = "REJECTED";
    cancel_response.reason = "Requested task ID does not match the active task.";
    cancel_response_publisher_->publish(cancel_response);
    return;
  }

  if (active_task) {
    active_task->request_cancel();
    // Publish cancellation response:
    publishCancelResponse(requested_task_id);
    RCLCPP_INFO(
      this->get_logger(), "UI cancel request: Task [%s] cancellation requested.",
      active_task_id.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "UI cancel request: Active task pointer is null.");
  }
}

// Utility to publish task rejection messages


// **Main Function**
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node and utility instance
  auto node = std::make_shared<rclcpp::Node>("autoware_bridge_node");
  auto autoware_bridge_util = std::make_shared<AutowareBridgeUtil>();
  std::atomic<bool> is_task_running(false);

  // Create task instances with correct arguments
  auto localization_task =
    std::make_shared<LocalizationTask>(node, autoware_bridge_util, is_task_running);
  auto set_goal_task = std::make_shared<SetGoalTask>(node, autoware_bridge_util, is_task_running);
  auto driving_task = std::make_shared<DrivingTask>(node, autoware_bridge_util, is_task_running);

  // Create AutowareBridgeNode
  auto bridge_node = std::make_shared<AutowareBridgeNode>(
    autoware_bridge_util, localization_task, set_goal_task, driving_task);

  rclcpp::spin(bridge_node);
  rclcpp::shutdown();
  return 0;
}
