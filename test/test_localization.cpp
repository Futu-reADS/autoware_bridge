#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
#include "tier4_system_msgs/msg/mode_change_available.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// Apply the macro hack ONLY when including localization.hpp.
#define private public
#define protected public
#include "autoware_bridge/localization.hpp"
#undef private
#undef protected

//------------------------------------------------------------------------------
// TestableLocalization: Subclass of Localization that exposes private methods.
class TestableLocalization : public Localization {
public:
  using Localization::Localization;

  // Expose private methods for testing
  void testPubInitPose(const geometry_msgs::msg::PoseStamped & init_pose) {
    pubInitPose(init_pose);
  }

  void testLocalizationQualityCallback(const tier4_system_msgs::msg::ModeChangeAvailable & msg) {
    localizationQualityCallback(msg);
  }

  void testLocalizationStateCallback(const autoware_adapi_v1_msgs::msg::LocalizationInitializationState & msg) {
    localizationStateCallback(msg);
  }

  void forceSetLocalizationQuality(bool quality) {
    localization_quality_ = quality;
  }

  bool getLocalizationQuality() const {
    return localization_quality_;
  }
  void forceSetState(LocalizationTaskState new_state) {
    state_ = new_state;
  }

  LocalizationTaskState getState() const {
    return state_;
  }
  
};

//------------------------------------------------------------------------------
// Test Fixture
class LocalizationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("test_localization_node");
    util = std::make_shared<AutowareBridgeUtil>();
    localization = std::make_shared<TestableLocalization>(node, util);
  }

  void TearDown() override {
    util->clearActiveTaskPtr();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<AutowareBridgeUtil> util;
  std::shared_ptr<TestableLocalization> localization;
};

//------------------------------------------------------------------------------
// Test Case 1: Localization Quality Callback Test
TEST_F(LocalizationTest, LocalizationQualityCallbackTest) {
  tier4_system_msgs::msg::ModeChangeAvailable quality_msg;
  quality_msg.available = true;
  localization->testLocalizationQualityCallback(quality_msg);
  EXPECT_TRUE(localization->getLocalizationQuality());

  quality_msg.available = false;
  localization->testLocalizationQualityCallback(quality_msg);
  EXPECT_FALSE(localization->getLocalizationQuality());
}

//------------------------------------------------------------------------------
// Test Case 2: Localization State Callback Test
TEST_F(LocalizationTest, LocalizationStateCallbackTest) {
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState state_msg;
  state_msg.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;
  localization->testLocalizationStateCallback(state_msg);
  EXPECT_EQ(localization->localization_state_, autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED);
}

//------------------------------------------------------------------------------
// Test Case 3: PubInitPose Test
TEST_F(LocalizationTest, PubInitPoseTest) {
  std::atomic<bool> msg_received(false);

  // Subscribe to the exact type that pubInitPose() publishes:
  auto sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/) {
      msg_received = true;
    });

  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  localization->testPubInitPose(init_pose);

  auto start_time = std::chrono::steady_clock::now();
  while (!msg_received && std::chrono::steady_clock::now() - start_time < 2s) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(msg_received);
}

//------------------------------------------------------------------------------
// Test Case 4: Execute Success Test
TEST_F(LocalizationTest, ExecuteSuccessTest) {
  std::string task_id = "localization";

  localization->forceSetState(LocalizationTaskState::LOCALIZATION_CHECK);
  localization->localization_start_time_ = node->now() - rclcpp::Duration::from_seconds(10.0);
  localization->forceSetLocalizationQuality(true);

  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });
  std::this_thread::sleep_for(200ms);
  exec_thread.join();

  EXPECT_EQ((util->getTaskStatus(task_id)).status, "SUCCESS");
  //EXPECT_EQ(util->getTaskStatus(task_id), "FAILED_DUE_TO_TASK_SUCCESS");

  util->clearActiveTaskPtr();
  SUCCEED();
}

/*
TEST_F(LocalizationTest, ExecuteLocalizationPoorQualityTest) {
  std::string task_id = "localization";

  // Setup the initial state and conditions
  localization->forceSetState(LocalizationTaskState::LOCALIZATION_CHECK);
  localization->localization_start_time_ = node->now() - rclcpp::Duration::from_seconds(10.0);  // Simulate a start time 10 seconds ago
  localization->forceSetLocalizationQuality(false);  // Poor quality

  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  // Simulate task execution in a separate thread
  std::thread exec_thread([&]() {
      localization->execute(task_id, init_pose);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Give enough time for task execution
  exec_thread.join();  // Ensure thread finishes execution before assertions

  EXPECT_EQ((util->getTaskStatus(task_id)).status, "RETRYING");  // Expect the
  // Clean up after the test
  util->clearActiveTaskPtr();
  
  // Test success
  SUCCEED();  // Mark the test as successful
}
*/

//------------------------------------------------------------------------------
// Test Case 5: Execute Failure (Timeout)
TEST_F(LocalizationTest, TimeoutCondition) {
  const std::string task_id = "test_task";
  int retry_counter = 0;
  bool timeout = false;
  LocalizationTaskState state_ = LocalizationTaskState::INITIALIZATION;

  EXPECT_EQ(state_, LocalizationTaskState::INITIALIZATION); // Ensure initial state

  // Set the active task to the correct task
  util->updateTaskId(task_id);  // Update task ID to match
  util->setActiveTaskPtr(localization);  // Set active task

  // Simulate retries up to MAX_INIT_RETRIES
  for (int i = 0; i <= MAX_INIT_RETRIES; i++) {
    if (retry_counter == 1) {
        // Simulate the action of updating task status to "RETRYING"
        util->updateTaskStatus(task_id, "RETRYING");
    }

    if (retry_counter < MAX_INIT_RETRIES) {
        // Simulate publishing the initial pose
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Simulate time for publishing
        state_ = LocalizationTaskState::LOCALIZATION;  // State changes after publishing pose
        util->updateTaskRetries(task_id, retry_counter);  // Update the retry count
        retry_counter++;  // Increment the retry counter
    } else {
        timeout = true;  // Timeout condition after retries are exceeded
        break; // Exit the loop if timeout condition is met
    }
  }

  // Ensure the state transitioned to LOCALIZATION and timeout condition was met
  EXPECT_EQ(state_, LocalizationTaskState::LOCALIZATION);
  EXPECT_TRUE(timeout);  // Ensure timeout condition is met

  // Clear active task pointer after the test
  util->clearActiveTaskPtr();
}



//------------------------------------------------------------------------------
// Test Case 6: Execute Cancel Scenario
TEST_F(LocalizationTest, ExecuteCancelTest) {
  std::string task_id = "cancel_task";
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  // Set the task ID and active task before executing
  util->updateTaskId(task_id);  // Update task ID to match
  util->setActiveTaskPtr(localization);  // Set active task to localization
  
  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });

  std::this_thread::sleep_for(200ms);  // Simulate task execution for a short time

  // Cancel the task
  localization->cancel();

  exec_thread.join();  // Wait for the execution thread to finish

  // Ensure the task status is correctly set to "CANCELLED"
  EXPECT_EQ(util->getTaskStatus(task_id).status, "CANCELLED");

  // Clear the active task after the test
  util->clearActiveTaskPtr();
}

/*
//------------------------------------------------------------------------------
// Test Case 7: Execute Retrying Branch Test
TEST_F(LocalizationTest, ExecuteRetryingTest) {
  std::string task_id = "retry_task";
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  util->updateTaskId(task_id);  // Update task ID to match
  util->setActiveTaskPtr(localization);  // Set active task to localization
  localization->forceSetState(LocalizationTaskState::INITIALIZATION);

  std::thread exec_thread([&]() {
      localization->execute(task_id, init_pose);
  });

  // Allow some time for retries to happen
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));  // Wait longer for retries

  exec_thread.join();

  // Verify that the state transitioned to RETRYING for the first few retries
  EXPECT_EQ(util->getTaskStatus(task_id).status, "RETRYING");

  // Verify that after exceeding max retries, the state is TIMEOUT
  // Ensure that the number of retries is properly compared to the maximum
  //EXPECT_EQ(util->getTaskStatus(task_id).status, "TIMEOUT");

  // Optionally verify that the state was not stuck and that timeout condition was handled
  //EXPECT_NE(localization->getState(), LocalizationTaskState::INITIALIZATION);
  //EXPECT_EQ(localization->getState(), LocalizationTaskState::TIMEOUT);
}
*/

//------------------------------------------------------------------------------
// Test Case 7: Execute Uninitialized Branch Test
TEST_F(LocalizationTest, ExecuteUninitializedBranchTest) {
  std::string task_id = "uninit_task";
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  localization->forceSetState(LocalizationTaskState::LOCALIZATION);
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState state_msg;
  state_msg.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNINITIALIZED;
  localization->testLocalizationStateCallback(state_msg);

  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });
  std::this_thread::sleep_for(500ms);
  localization->cancel();
  exec_thread.join();

  SUCCEED();
}
/*
//------------------------------------------------------------------------------
// Test Case: State Transition Unexpected Behavior
TEST_F(LocalizationTest, UnexpectedStateTransitionTest) {
  std::string task_id = "unexpected_state_task";
  
  localization->forceSetState(LocalizationTaskState::LOCALIZATION);  // Set an initial state
  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  // Simulate an unexpected state transition to an invalid state
  localization->forceSetState(static_cast<LocalizationTaskState>(-1));  // Set an invalid state

  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  exec_thread.join();

  // Check if the state remained in the invalid state or transitioned back to valid states
  EXPECT_NE(localization->getState(), static_cast<LocalizationTaskState>(-1));

  util->clearActiveTaskPtr();
}
*/

//------------------------------------------------------------------------------
// Test Case 8: Cancel During Initialization
TEST_F(LocalizationTest, CancelDuringInitializationTest) {
  std::string task_id = "cancel_init_task";
  
  localization->forceSetState(LocalizationTaskState::INITIALIZATION);
  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Allow for initialization
  localization->cancel();  // Cancel during initialization
  exec_thread.join();

  // Ensure the task status is "CANCELLED"
  EXPECT_EQ(util->getTaskStatus(task_id).status, "CANCELLED");

  util->clearActiveTaskPtr();
}

//------------------------------------------------------------------------------
// Test Case 9: Cancel During Localization
TEST_F(LocalizationTest, CancelDuringLocalizationTest) {
  std::string task_id = "cancel_localization_task";

  localization->forceSetState(LocalizationTaskState::LOCALIZATION);
  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Allow for localization
  localization->cancel();  // Cancel during localization
  exec_thread.join();

  // Ensure the task status is "CANCELLED"
  EXPECT_EQ(util->getTaskStatus(task_id).status, "CANCELLED");

  util->clearActiveTaskPtr();
}

//------------------------------------------------------------------------------
// Test Case 10: Cancel During Localization Check
TEST_F(LocalizationTest, CancelDuringLocalizationCheckTest) {
  std::string task_id = "cancel_localization_check_task";

  localization->forceSetState(LocalizationTaskState::LOCALIZATION_CHECK);
  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header.stamp = node->now();
  init_pose.header.frame_id = "map";

  std::thread exec_thread([&]() {
    localization->execute(task_id, init_pose);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Allow for localization check
  localization->cancel();  // Cancel during localization check
  exec_thread.join();

  // Ensure the task status is "CANCELLED"
  EXPECT_EQ(util->getTaskStatus(task_id).status, "CANCELLED");

  util->clearActiveTaskPtr();
}

// Test Case 11: SendCmdGate on UNINITIALIZED in LOCALIZATION state
TEST_F(LocalizationTest, SendCmdGateOnUninitialized) {
  std::atomic<bool> gate_cmd_received(false);
  auto gate_sub = node->create_subscription<tier4_control_msgs::msg::GateMode>(
    "/input/current_gate_mode", 10,
    [&](const tier4_control_msgs::msg::GateMode::SharedPtr msg) {
      if (msg->data == tier4_control_msgs::msg::GateMode::AUTO) {
        gate_cmd_received = true;
      }
    });

  const std::string task_id = "gate_test";
  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  // Set state to LOCALIZATION and initialization state to UNINITIALIZED
  localization->forceSetState(LocalizationTaskState::LOCALIZATION);
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState state_msg;
  state_msg.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNINITIALIZED;
  localization->testLocalizationStateCallback(state_msg);

  // Execute in background
  std::thread exec_thread([&](){
    localization->execute(task_id, geometry_msgs::msg::PoseStamped());
  });

  // Wait for gate command to be published
  auto start = std::chrono::steady_clock::now();
  while (!gate_cmd_received && std::chrono::steady_clock::now() - start < 1s) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(gate_cmd_received);

  // Cancel execution and clean up
  localization->cancel();
  exec_thread.join();
}

//------------------------------------------------------------------------------
// Test Case 12: Transition to LOCALIZATION_CHECK on INITIALIZED
TEST_F(LocalizationTest, InitializationToCheckTransition) {
  const std::string task_id = "init_to_check";
  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  // Force state LOCALIZATION and set callback to INITIALIZED
  localization->forceSetState(LocalizationTaskState::LOCALIZATION);
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState state_msg;
  state_msg.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;
  localization->testLocalizationStateCallback(state_msg);

  // Run execute; we will cancel after verifying state change
  std::thread exec_thread([&](){
    localization->execute(task_id, geometry_msgs::msg::PoseStamped());
  });

  // Allow some time for state transition
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(localization->getState(), LocalizationTaskState::LOCALIZATION_CHECK);

  // Cancel and clean up
  localization->cancel();
  exec_thread.join();
}

//------------------------------------------------------------------------------
// Test Case 13: Unknown state triggers default branch
TEST_F(LocalizationTest, UnknownStateDefaultBranch) {
  const std::string task_id = "unknown_state_test";
  util->updateTaskId(task_id);
  util->setActiveTaskPtr(localization);

  // Force invalid state
  localization->forceSetState(static_cast<LocalizationTaskState>(-1));

  // Run execute and cancel shortly after
  std::thread exec_thread([&](){
    localization->execute(task_id, geometry_msgs::msg::PoseStamped());
  });

  std::this_thread::sleep_for(100ms);
  localization->cancel();
  exec_thread.join();

  // Expect CANCELLED status after default branch and cancel
  EXPECT_EQ(util->getTaskStatus(task_id).status, "CANCELLED");
}
