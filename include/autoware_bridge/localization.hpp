#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "base_task.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>

#include <atomic>
#include <memory>
#include <mutex>

enum class LocalizationTaskState { INITIALIZATION, LOCALIZATION, LOCALIZATION_CHECK };

const double LOC_WAIT_TIMEOUT_S = 10.0;
const int MAX_INIT_RETRIES = 2;

class Localization : public BaseTask
{
public:
  Localization(
    std::shared_ptr<rclcpp::Node> node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util);

  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & init_pose)
    override;                           // Executes localization
  void cancel() override;               // Requests task cancellation

  // Alias
  using LocalizationInitializationState =
    autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using GateMode = tier4_control_msgs::msg::GateMode;

private:
  //rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;
  std::atomic<bool> is_cancel_requested_;

  LocalizationTaskState state_;
  uint16_t localization_state_;
  bool localization_quality_;
  rclcpp::Time localization_start_time_;

  std::mutex task_mutex_;

  // Helper methods
  void sendCmdGate();
  void pubInitPose(const geometry_msgs::msg::PoseStamped & init_pose);

  // callbacks
  void localizationQualityCallback(const ModeChangeAvailable & msg);
  void localizationStateCallback(const LocalizationInitializationState & msg);

  // Publisher
  //rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr init_pose_publisher_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr init_pose_publisher_;
  rclcpp::Publisher<GateMode>::SharedPtr current_gate_mode_publisher_;

  // Subscriber
  rclcpp::Subscription<LocalizationInitializationState>::SharedPtr localization_state_subscriber_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr localization_quality_subscriber_;
};

#endif  // LOCALIZATION_HPP
