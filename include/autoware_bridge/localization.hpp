#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "base_task.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <atomic>
#include <memory>

enum class LocalizationTaskState {
  UNINITIALIZED,
  INITIALIZATION,
  LOCALIZATION,
  LOCALIZATION_CHECK
};

enum class LocalizationInitializationState { UNKNOWN, UNINITIALIZED, INITIALIZING, INITIALIZED };

class Localization : public BaseTask
{
public:
  Localization(
    rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
    std::atomic<bool> & is_task_running);

  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & init_pose)
    override;                      // Executes localization
  void request_cancel() override;  // Requests task cancellation
  bool isLocalizationQualityAcceptable() const;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;  // Use shared_ptr instead of reference
  std::atomic<bool> cancel_requested_;
  std::atomic<bool> & is_task_running_;

  LocalizationTaskState state_;
  LocalizationInitializationState loc_state_;
  bool localization_quality_;
  std::chrono::steady_clock::time_point localization_start_time_;

  // Timeout threshold (in seconds) for localization processing.
  const double LOC_WAIT_TIMEOUT_S = 10.0;

  // Helper methods
  void sendCmdGate();
  void pubInitPose(const geometry_msgs::msg::PoseStamped & init_pose);

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_publisher_;
};

#endif  // LOCALIZATION_HPP
