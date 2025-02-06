#ifndef AUTOWARE_BRIDGE_HPP
#define AUTOWARE_BRIDGE_HPP

#include <autoware_bridge/srv/get_task_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

class AutowareBridgeNode : public rclcpp::Node
{
public:
  AutowareBridgeNode();
  ~AutowareBridgeNode();

private:
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;

  // Service for querying task status
  rclcpp::Service<autoware_bridge::srv::GetTaskStatus>::SharedPtr status_service_;

  // Task Queue and Thread Management
  std::queue<std::pair<std::string, std::function<void()>>> task_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread executor_thread_;
  bool stop_executor_ = false;

  // Task Status Tracking
  std::unordered_map<std::string, std::string> task_status_;
  std::mutex task_mutex_;

  // Callback Functions
  void topic_callback_1(const std_msgs::msg::String::SharedPtr msg);
  void topic_callback_2(const std_msgs::msg::String::SharedPtr msg);

  // Task Processing Functions
  void localization(const std::string & task_id);
  void set_goal(const std::string & task_id);

  // Executor Thread Function
  void task_executor();

  // Utility Functions
  void update_task_status(const std::string & task_id, const std::string & status);
  std::string generate_task_id(const std::string & prefix);

  // Service Handler
  void handle_status_request(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus::Response> response);
};

#endif  // AUTOWARE_BRIDGE_HPP
