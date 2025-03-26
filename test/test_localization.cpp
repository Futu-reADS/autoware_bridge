// test_localization.cpp

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_bridge/localization.hpp"
#include "autoware_bridge/autoware_bridge_util.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// For testing, these constants should match your production definitions.
//static constexpr int MAX_INIT_RETRIES = 3;
//static constexpr double LOC_WAIT_TIMEOUT_S = 1.0;

// -----------------------------------------------------------------------------
// DummyBridgeUtil: a simple subclass of AutowareBridgeUtil that records calls.
class DummyBridgeUtil : public AutowareBridgeUtil {
public:
  std::vector<std::string> call_log;
  std::mutex log_mutex;

  void updateTaskStatus(const std::string & task_id, const std::string & status, std::string reason = "") {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateTaskStatus: " + task_id + " status:" + status + " reason:" + reason);
    }
    // Call base implementation.
    AutowareBridgeUtil::updateTaskStatus(task_id, status, reason);
  }

  void updateTaskRetries(const std::string & task_id, int retry_count) {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateTaskRetries: " + task_id + " retry:" + std::to_string(retry_count));
    }
    AutowareBridgeUtil::updateTaskRetries(task_id, retry_count);
  }
  
  void updateTask(const std::string & task_id, TaskRequestType type, const std::string & value, int number = 0) {
    {
      std::lock_guard<std::mutex> lock(log_mutex);
      call_log.push_back("updateTask: " + task_id +
                          " type:" + std::to_string(static_cast<int>(type)) +
                          " value:" + value +
                          " num:" + std::to_string(number));
    }
    AutowareBridgeUtil::updateTask(task_id, type, value, number);
  }
};

// -----------------------------------------------------------------------------
// TestableLocalization: a subclass of Localization that exposes internal members
// for testing. In your production header, mark the following as protected (or friend the test):
//   - pubInitPose()
//   - localizationQualityCallback()
//   - localizationStateCallback()
//   - localization_start_time_
//   - state_
class TestableLocalization : public Localization {
public:
  using Localization::Localization;  // Inherit constructors

  // Public wrapper for pubInitPose.
  void testPubInitPose(const geometry_msgs::msg::PoseStamped & init_pose) {
    pubInitPose(init_pose);
  }

  // Public wrappers for callbacks.
  void testLocalizationQualityCallback(const tier4_system_msgs::msg::ModeChangeAvailable & msg) {
    localizationQualityCallback(msg);
  }
  
  void testLocalizationStateCallback(const autoware_adapi_v1_msgs::msg::LocalizationInitializationState & msg) {
    localizationStateCallback(msg);
  }
  
  // Public setter and getter for localization_start_time_.
  void setLocalizationStartTime(const rclcpp::Time & time) {
    localization_start_time_ = time;
  }
  rclcpp::Time getLocalizationStartTime() const {
    return localization_start_time_;
  }
  
  // Expose a method to force internal state changes.
  void forceSetLocalizationQuality(bool quality) {
    localization_quality_ = quality;
  }
  void forceSetState(LocalizationTaskState new_state) {
    state_ = new_state;
  }
};

class LocalizationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("test_localization_node");
    is_task_running = false;
    dummy_util = std::make_shared<DummyBridgeUtil>();
    localization = std::make_shared<TestableLocalization>(node, dummy_util);
  }
  
  void TearDown() override {
    rclcpp::shutdown();
  }
  
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<DummyBridgeUtil> dummy_util;
  std::atomic<bool> is_task_running;
  std::shared_ptr<TestableLocalization> localization;
};

// -----------------------------------------------------------------------------
// Test Cases

// 1. Test Localization Quality Callback.
TEST_F(LocalizationTest, QualityCallbackTest) {
  tier4_system_msgs::msg::ModeChangeAvailable quality_msg;
  quality_msg.available = true;
  localization->testLocalizationQualityCallback(quality_msg);
  EXPECT_TRUE(localization->getLocalizationQuality());

  quality_msg.available = false;
  localization->testLocalizationQualityCallback(quality_msg);
  EXPECT_FALSE(localization->getLocalizationQuality());
}

// 2. Test Localization State Callback.
TEST_F(LocalizationTest, StateCallbackTest) {
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState state_msg;
  state_msg.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;
  localization->testLocalizationStateCallback(state_msg);
  SUCCEED();  // No error implies success.
}

// 3. Test PubInitPose: Verify that a message is published on "/initialpose".
TEST_F(LocalizationTest, PubInitPoseTest) {
  std::atomic<bool> msg_received{false};
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/initialpose", 10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr) {
      msg_received = true;
    });
  
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  localization->testPubInitPose(init_pose);
  rclcpp::spin_some(node);
  EXPECT_TRUE(msg_received);
}

// 4. Test Execute Success Scenario.
TEST_F(LocalizationTest, ExecuteSuccessTest) {
  std::string task_id = "success_task";
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  // Force conditions for success:
  // - Set quality to true.
  // - Set start time in the past to trigger a timeout in LOCALIZATION_CHECK.
  localization->forceSetLocalizationQuality(true);
  localization->setLocalizationStartTime(node->now() - rclcpp::Duration::from_seconds(LOC_WAIT_TIMEOUT_S + 1));
  // Force state to LOCALIZATION_CHECK.
  localization->forceSetState(LocalizationTaskState::LOCALIZATION_CHECK);
  
  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });
  exec_thread.join();
  
  bool successLogged = false;
  for (const auto & entry : dummy_util->call_log) {
    if (entry.find("updateTaskStatus: " + task_id + " status:SUCCESS") != std::string::npos) {
      successLogged = true;
      break;
    }
  }
  EXPECT_TRUE(successLogged);
}

// 5. Test Execute Failure Scenario (max retries exceeded).
TEST_F(LocalizationTest, ExecuteFailureTest) {
  std::string task_id = "fail_task";
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  // Force failure: set quality to false so that success is never reached.
  localization->forceSetLocalizationQuality(false);
  // Set start time in the past.
  localization->setLocalizationStartTime(node->now() - rclcpp::Duration::from_seconds(LOC_WAIT_TIMEOUT_S + 1));
  localization->forceSetState(LocalizationTaskState::LOCALIZATION_CHECK);
  
  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });
  exec_thread.join();
  
  bool failedLogged = false;
  for (const auto & entry : dummy_util->call_log) {
    if (entry.find("updateTaskStatus: " + task_id + " status:FAILED") != std::string::npos &&
        entry.find("Max retries elapsed") != std::string::npos) {
      failedLogged = true;
      break;
    }
  }
  EXPECT_TRUE(failedLogged);
}

// 6. Test Execute Cancellation Scenario.
TEST_F(LocalizationTest, ExecuteCancelTest) {
  std::string task_id = "cancel_task";
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";
  
  // Start execute() in a separate thread.
  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });
  
  std::this_thread::sleep_for(200ms);
  localization->cancel();
  exec_thread.join();
  
  bool cancelLogged = false;
  for (const auto & entry : dummy_util->call_log) {
    if (entry.find("updateTaskStatus: " + task_id + " status:CANCELLED") != std::string::npos) {
      cancelLogged = true;
      break;
    }
  }
  EXPECT_TRUE(cancelLogged);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
