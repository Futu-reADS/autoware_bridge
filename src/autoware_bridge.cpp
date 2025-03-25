#include "autoware_bridge/autoware_bridge.hpp"

#include <functional>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

AutowareBridgeNode::AutowareBridgeNode(
    std::shared_ptr<AutowareBridgeUtil> util)
    : Node("autoware_bridge_node"),
      autoware_bridge_util_(util),
      is_task_running_(false)
{
  this->declare_parameter("localization_topic", "/ftd_master/localization_request");
  this->declare_parameter("route_planning_topic", "/ftd_master/route_planning_request");
  this->declare_parameter("autonomous_driving_topic", "/ftd_master/autonomous_driving_request");
  this->declare_parameter("cancel_task_topic", "/ftd_master/cancel_task");

  std::string localization_topic = this->get_parameter("localization_topic").as_string();
  std::string route_planning_topic = this->get_parameter("route_planning_topic").as_string();
  std::string autonomous_driving_topic = this->get_parameter("autonomous_driving_topic").as_string();
  std::string cancel_task_topic = this->get_parameter("cancel_task_topic").as_string();

  // ROS2 Subscriptions
  localization_request_subscription_ =
      this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(localization_topic, 10, std::bind(&AutowareBridgeNode::localizationRequestCallback, this, std::placeholders::_1));

  route_planning_request_subscription_ =
      this->create_subscription<ftd_master_msgs::msg::PoseStampedWithTaskId>(route_planning_topic, 10, std::bind(&AutowareBridgeNode::routePlanningRequestCallback, this, std::placeholders::_1));

  autonomous_driving_request_subscription_ =
      this->create_subscription<std_msgs::msg::String>(autonomous_driving_topic, 10,
                                                       std::bind(&AutowareBridgeNode::autonomousDrivingRequestCallback, this, std::placeholders::_1));

  cancel_task_subscription_ =
      this->create_subscription<std_msgs::msg::String>(cancel_task_topic, 10,
                                                       std::bind(&AutowareBridgeNode::cancelTaskCallback, this, std::placeholders::_1));

  // ROS2 Publishers
  task_response_publisher_ = this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
      "/autoware_bridge/task_response", 10);

  cancel_response_publisher_ =
      this->create_publisher<autoware_bridge_msgs::msg::TaskStatusResponse>(
          "/autoware_bridge/cancel_task_response", 10);

  // ROS2 Services
  status_service_ = this->create_service<autoware_bridge::srv::GetTaskStatus>(
      "check_task_status", std::bind(
                               &AutowareBridgeNode::handleStatusRequest, this, std::placeholders::_1,
                               std::placeholders::_2));

  // Create a timer to check localization quality periodically.
  timer_ = this->create_wall_timer(100ms, std::bind(&AutowareBridgeNode::onTimerCallback, this));

  reinitialize_response_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/autoware_bridge/reinitialize", 10);

  localization_quality_subscriber_ = this->create_subscription<ModeChangeAvailable>(
      "/system/component_state_monitor/component/autonomous/localization",
      rclcpp::QoS(1).transient_local(),
      std::bind(&AutowareBridgeNode::localizationQualityCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Autoware Bridge Node has been initialized.");
}

AutowareBridgeNode::~AutowareBridgeNode()
{
  // Optionally: join or signal threads if you decide to track them instead of detaching.
}

// Task Handling Callbacks
void AutowareBridgeNode::localizationRequestCallback(
    const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{
  if (isTaskRejected("localization"))
  {
    return;
  }

  auto node_ptr = std::enable_shared_from_this<AutowareBridgeNode>::shared_from_this();
  auto localization_task = std::make_shared<Localization>(node_ptr, autoware_bridge_util_);

  startTaskExecution(msg->task_id.data, msg->pose_stamped, localization_task);
}

void AutowareBridgeNode::routePlanningRequestCallback(
    const ftd_master_msgs::msg::PoseStampedWithTaskId::SharedPtr msg)
{
  if (isTaskRejected("route_planning"))
  {
    return;
  }
  auto node_ptr = std::enable_shared_from_this<AutowareBridgeNode>::shared_from_this();
  auto route_planning_task = std::make_shared<RoutePlanning>(node_ptr, autoware_bridge_util_);
  startTaskExecution(msg->task_id.data, msg->pose_stamped, route_planning_task);
}

void AutowareBridgeNode::autonomousDrivingRequestCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
  if (isTaskRejected("autonomous_driving"))
  {
    return;
  }
  geometry_msgs::msg::PoseStamped dummy_pose_stamped;
  auto node_ptr = std::enable_shared_from_this<AutowareBridgeNode>::shared_from_this();
  auto autonomous_driving_task = std::make_shared<AutonomousDriving>(node_ptr, autoware_bridge_util_);
  startTaskExecution(msg->data, dummy_pose_stamped, autonomous_driving_task);
}

bool AutowareBridgeNode::isTaskRejected(const std::string &task_name)
{
  if (is_task_running_.exchange(true))
  {
    RCLCPP_WARN(
        this->get_logger(), "A task is already running. %s request rejected.", task_name.c_str());
    publishTaskRejectionReason(task_name);
    return true;
  }
  return false;
}

void AutowareBridgeNode::startTaskExecution(
    const std::string &requested_task_id, const geometry_msgs::msg::PoseStamped &pose_stamped,
    std::shared_ptr<BaseTask> task)
{
  RCLCPP_INFO(this->get_logger(), "Start task_id: %s", requested_task_id.c_str());
  autoware_bridge_util_->updateTaskId(requested_task_id);
  autoware_bridge_util_->updateTaskStatus(requested_task_id, "PENDING");
  autoware_bridge_util_->setActiveTask(task);
  startThreadExecution(requested_task_id, pose_stamped);
}

void AutowareBridgeNode::startThreadExecution(
    const std::string &requested_task_id, const geometry_msgs::msg::PoseStamped &pose_stamped)
{
  std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->getActiveTaskPointer();
  if (active_task)
  {
    std::thread([this, active_task, requested_task_id, pose_stamped]()
                {
      active_task->execute(requested_task_id, pose_stamped);
      
      is_task_running_ = false;
      std::lock_guard<std::mutex> lock(task_mutex_);
      autoware_bridge_util_->clearActiveTask();
      publishTaskResponse(requested_task_id); })
        .detach();
  }
  else
  {
    RCLCPP_ERROR(
        this->get_logger(), "Active task pointer is null for task_id: %s", requested_task_id.c_str());
  }
}

void AutowareBridgeNode::publishTaskRejectionReason(const std::string &task_name)
{
  std::string active_task_id = autoware_bridge_util_->getActiveTaskId();
  if (active_task_id == "NO_ACTIVE_TASK")
  {
    RCLCPP_WARN(
        this->get_logger(), "No active task is currently running. Ignoring %s request.",
        task_name.c_str());
    return;
  }
  else
  {
    autoware_bridge_msgs::msg::TaskStatusResponse failure_msg;
    failure_msg.task_id = task_name;
    failure_msg.status = "REJECTED";
    failure_msg.reason =
        "Task rejected: " + task_name + " request ignored because " + active_task_id + " is running.";
    task_response_publisher_->publish(failure_msg);
  }
}

void AutowareBridgeNode::publishTaskResponse(const std::string &task_id)
{
  if (autoware_bridge_util_->isTaskActive(task_id))
  {
    TaskInfo task_status = autoware_bridge_util_->getTaskStatus(task_id);
    autoware_bridge_msgs::msg::TaskStatusResponse task_response =
        createTaskStatusResponse(task_id, task_status.status, task_status.reason);
    task_response_publisher_->publish(task_response);
  }
  else
  {
    RCLCPP_WARN(
        this->get_logger(), "Requested task_id: %s is not the active one.", task_id.c_str());
  }
}

void AutowareBridgeNode::cancelTaskCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  std::string requested_task_id = msg->data;
  if (autoware_bridge_util_->isTaskActive(requested_task_id))
  {

    TaskInfo task_status = autoware_bridge_util_->getTaskStatus(requested_task_id);
    if (task_status.status == "FAILED" || task_status.status == "SUCCESS")
    {
      publishCancelResponse(requested_task_id);
      return;
    }

    autoware_bridge_util_->updateCancellationStatus(requested_task_id, "REQUESTED");

    std::shared_ptr<BaseTask> active_task = autoware_bridge_util_->getActiveTaskPointer();
    if (active_task)
    {
      active_task->cancelRequested();
      TaskCancellationInfo task_cancellation_status =
          autoware_bridge_util_->getTaskStatus(requested_task_id).cancel_info;
      while (task_cancellation_status.status == "REQUESTED")
      {
        // some delay (10ms)
      }

      publishCancelResponse(requested_task_id);
    }
    else
    {
      publishCancelResponse(requested_task_id);
      RCLCPP_ERROR(
          this->get_logger(), "task_id: %s cancel request failed as active task pointer is null.",
          requested_task_id.c_str());
    }
  }
  else
  {
    autoware_bridge_msgs::msg::TaskStatusResponse cancel_response;
    cancel_response.task_id = requested_task_id;
    cancel_response.status = "REJECTED";
    cancel_response.reason = "Requested task ID is not the active task.";
    cancel_response_publisher_->publish(cancel_response);
  }
}

void AutowareBridgeNode::publishCancelResponse(const std::string &task_id)
{
  if (autoware_bridge_util_->isTaskActive(task_id))
  {
    TaskCancellationInfo task_cancellation_status =
        autoware_bridge_util_->getTaskStatus(task_id).cancel_info;
    autoware_bridge_msgs::msg::TaskStatusResponse cancel_response = createTaskStatusResponse(
        task_id, task_cancellation_status.status, task_cancellation_status.reason);
    cancel_response_publisher_->publish(cancel_response);
  }
  else
  {
    RCLCPP_WARN(
        this->get_logger(), "Requested task_id: %s is not the active one to cancel.",
        task_id.c_str());
  }
}

autoware_bridge_msgs::msg::TaskStatusResponse AutowareBridgeNode::createTaskStatusResponse(
    const std::string &task_id, const std::string &status, const std::string &reason)
{
  autoware_bridge_msgs::msg::TaskStatusResponse response;
  response.task_id = task_id;
  response.status = status;
  if (response.status != "SUCCESS")
  {
    response.reason = reason;
  }
  else
  {
    response.reason = "";
  }
  return response;
}

void AutowareBridgeNode::handleStatusRequest(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response)
{
  autoware_bridge_util_->handleStatusRequest(request, response);
}

void AutowareBridgeNode::onTimerCallback()
{
  std::string active_task_id = autoware_bridge_util_->getActiveTaskId();
  TaskInfo task_status = autoware_bridge_util_->getTaskStatus(active_task_id);

  if ((((active_task_id.find("localization") == 0) && (task_status.status == "SUCCESS"))) ||
      ((active_task_id.find("route_planning") == 0)) ||
      ((active_task_id.find("autonomous_driving") == 0)))
  {
    if (!localization_quality_)
    {
      // trigger reinitialization to UI.
      std_msgs::msg::Bool reinit_msg;
      reinit_msg.data = true;
      reinitialize_response_publisher_->publish(reinit_msg);
      RCLCPP_WARN(get_logger(), "Localization quality is poor,UI should request for reinitialization");
    }
  }
}

void AutowareBridgeNode::localizationQualityCallback(const ModeChangeAvailable &msg)
{
  localization_quality_ = msg.available;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create utility instance and node instance
  auto autoware_bridge_util = std::make_shared<AutowareBridgeUtil>();

  // Create AutowareBridgeNode
  auto bridge_node = std::make_shared<AutowareBridgeNode>(autoware_bridge_util);

  rclcpp::spin(bridge_node);
  rclcpp::shutdown();
  return 0;
}
