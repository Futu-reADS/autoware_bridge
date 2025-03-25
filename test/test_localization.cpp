// test_localization.cpp

#include <gtest/gtest.h>
#include "autoware_bridge/localization.hpp"
#include "autoware_bridge/autoware_bridge_util.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>

// --- Dummy definitions for missing types and constants ---

// These enums must match your production definitions.
enum class LocalizationTaskState { INITIALIZATION, LOCALIZATION, LOCALIZATION_CHECK };
enum class LocalizationInitializationState { UNKNOWN, UNINITIALIZED, INITIALIZED };

// Define maximum initialization retries and localization timeout for testing.
const int MAX_INIT_RETRIES = 3;
const double LOC_WAIT_TIMEOUT_S = 1.0;

// Dummy message for mode change availability.
struct ModeChangeAvailable {
  bool available;
};

// Mimic LocalizationInitializationState message.
struct DummyLocalizationInitializationState {
  int state; // reinterpret as LocalizationInitializationState when needed.
};

// --- TestAutowareBridgeUtil --------------------------------------------------
// A dummy subclass of AutowareBridgeUtil that records calls for verification.

class TestAutowareBridgeUtil : public AutowareBridgeUtil {
public:
  std::vector<std::string> call_log;
  std::mutex log_mutex;

  // Override methods to record calls.
  void updateTaskStatus(
    const std::string & task_id,
    TaskRequestType request_type,
    const std::string & value,
    int number = 0) override
  {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateTaskStatus: " + task_id +
                           " type:" + std::to_string(static_cast<int>(request_type)) +
                           " value:" + value + " num:" + std::to_string(number));
    }
    AutowareBridgeUtil::updateTaskStatus(task_id, request_type, value, number);
  }

  void updateRunningStatusWithRetries(const std::string & task_id, const int total_retries) override {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateRunningStatusWithRetries: " + task_id +
                           " total_retries:" + std::to_string(total_retries));
    }
    AutowareBridgeUtil::updateRunningStatusWithRetries(task_id, total_retries);
  }

  void updateCancellationStatus(const std::string & task_id, const std::string & reason) override {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateCancellationStatus: " + task_id + " reason:" + reason);
    }
    AutowareBridgeUtil::updateCancellationStatus(task_id, reason);
  }

  void updateFailStatus(const std::string & task_id, const std::string & reason) override {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateFailStatus: " + task_id + " reason:" + reason);
    }
    AutowareBridgeUtil::updateFailStatus(task_id, reason);
  }

  void updateSuccessStatus(const std::string & task_id) override {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateSuccessStatus: " + task_id);
    }
    AutowareBridgeUtil::updateSuccessStatus(task_id);
  }
};

// --- Test Fixture for Localization -----------------------------------------

class LocalizationTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize ROS2.
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("test_localization_node");
    is_task_running = false;
    util = std::make_shared<TestAutowareBridgeUtil>();

    // Create Localization instance.
    localization = std::make_shared<Localization>(node, util, std::ref(is_task_running));
  }
  
  void TearDown() override {
    rclcpp::shutdown();
  }
  
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<TestAutowareBridgeUtil> util;
  std::atomic<bool> is_task_running;
  std::shared_ptr<Localization> localization;
};

// --- Test Cases --------------------------------------------------------------

// 1. Test the callback for localization quality.
/* TEST_F(LocalizationTest, LocalizationQualityCallbackTest) {
  ModeChangeAvailable msg;
  msg.available = true;
  localization->localizationQualityCallback(msg);
  EXPECT_TRUE(localization->getLocalizationQuality());
  
  msg.available = false;
  localization->localizationQualityCallback(msg);
  EXPECT_FALSE(localization->getLocalizationQuality());
}

// 2. Test the callback for localization state.
TEST_F(LocalizationTest, LocalizationStateCallbackTest) {
  DummyLocalizationInitializationState msg;
  msg.state = static_cast<int>(LocalizationInitializationState::INITIALIZED);
  localization->localizationStateCallback(*(reinterpret_cast<const LocalizationInitializationState*>(&msg)));
  // If no error occurs, the test succeeds.
  SUCCEED();
}

// 3. Test that pubInitPose publishes a message on the expected topic.
TEST_F(LocalizationTest, PubInitPoseTest) {
  std::atomic<bool> msg_received{false};
  
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/initialpose", 10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr /*msg) {
      msg_received = true;
    });
  
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  localization->pubInitPose(init_pose);
  
  // Process callbacks.
  rclcpp::spin_some(node);
  EXPECT_TRUE(msg_received);
}

// 4. Test cancellation: run execute() in a thread and then cancel.
TEST_F(LocalizationTest, CancellationTest) {
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  std::thread exec_thread([&]() {
    localization->execute("cancel_task", init_pose);
  });
  
  // Let the execution run briefly.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // Request cancellation.
  localization->cancelRequested();
  
  exec_thread.join();
  
  // Verify that updateCancellationStatus was called.
  bool cancellation_logged = false;
  {
    std::lock_guard<std::mutex> lock(util->log_mutex);
    for (const auto & entry : util->call_log) {
      if (entry.find("updateCancellationStatus: cancel_task") != std::string::npos) {
        cancellation_logged = true;
        break;
      }
    }
  }
  EXPECT_TRUE(cancellation_logged);
  // Also check that task_running flag is false.
  EXPECT_FALSE(is_task_running);
}

// 5. Test failure scenario: force max retries reached (simulate no state change).
TEST_F(LocalizationTest, FailureTest) {
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  // Do not update state/quality so success never occurs.
  std::thread exec_thread([&]() {
    localization->execute("fail_task", init_pose);
  });
  
  exec_thread.join();
  
  bool fail_logged = false;
  {
    std::lock_guard<std::mutex> lock(util->log_mutex);
    for (const auto & entry : util->call_log) {
      if (entry.find("updateFailStatus: fail_task") != std::string::npos) {
        fail_logged = true;
        break;
      }
    }
  }
  EXPECT_TRUE(fail_logged);
  EXPECT_FALSE(is_task_running);
}

// 6. Test success scenario: simulate a successful localization.
// Force the necessary state transitions and quality.
TEST_F(LocalizationTest, SuccessTest) {
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  // Simulate state transition to INITIALIZED.
  DummyLocalizationInitializationState state_msg;
  state_msg.state = static_cast<int>(LocalizationInitializationState::INITIALIZED);
  localization->localizationStateCallback(*(reinterpret_cast<const LocalizationInitializationState*>(&state_msg)));
  
  // Simulate good localization quality.
  ModeChangeAvailable quality_msg;
  quality_msg.available = true;
  localization->localizationQualityCallback(quality_msg);
  
  // Force start time in the past to trigger the LOCALIZATION_CHECK timeout.
  localization->localization_start_time_ = node->now() - rclcpp::Duration::from_seconds(LOC_WAIT_TIMEOUT_S + 1);
  
  std::thread exec_thread([&]() {
    localization->execute("success_task", init_pose);
  });
  
  exec_thread.join();
  
  bool success_logged = false;
  {
    std::lock_guard<std::mutex> lock(util->log_mutex);
    for (const auto & entry : util->call_log) {
      if (entry.find("updateSuccessStatus: success_task") != std::string::npos) {
        success_logged = true;
        break;
      }
    }
  }
  EXPECT_TRUE(success_logged);
  EXPECT_FALSE(is_task_running);
}

// 7. (Optional) Test for unexpected state: publish an invalid state value and ensure no success.
TEST_F(LocalizationTest, UnexpectedStateTest) {
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  std::string task_id = "unexpected_task";
  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });
  
  // Wait a bit, then publish an invalid state.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  DummyLocalizationInitializationState invalid_state;
  invalid_state.state = 99; // Invalid state
  localization->localizationStateCallback(*(reinterpret_cast<const LocalizationInitializationState*>(&invalid_state)));
  
  // Let it run for a short period.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // Since state is invalid, expect that neither success nor failure update is called,
  // but eventually the max retry condition will force a failure.
  exec_thread.join();
  
  bool fail_logged = false;
  {
    std::lock_guard<std::mutex> lock(util->log_mutex);
    for (const auto & entry : util->call_log) {
      if (entry.find("updateFailStatus: unexpected_task") != std::string::npos) {
        fail_logged = true;
        break;
      }
    }
  }
  EXPECT_TRUE(fail_logged);
  EXPECT_FALSE(is_task_running);
}
 */