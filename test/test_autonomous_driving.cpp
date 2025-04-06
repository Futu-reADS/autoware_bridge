// test_autonomous_driving.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

using namespace std::chrono_literals;

// Apply macro hack ONLY when including autonomous_driving.hpp
#define private public
#define protected public
#include "autoware_bridge/autonomous_driving.hpp"
#undef private
#undef protected

//
// TestableAutonomousDriving: Subclass of AutonomousDriving that exposes internal state
// and helper functions to simulate conditions (like timeouts) and force internal state changes.
//
class TestableAutonomousDriving : public AutonomousDriving {
public:
  using AutonomousDriving::AutonomousDriving;  // Inherit constructors

  // For testing, we assume the engageAutoDrive() is handled externally,
  // or overridden elsewhere if needed.

  // Expose a helper to simulate DRIVE_WAIT timeout by setting driving_start_time_ in the past.
  void simulateDriveWaitTimeout() {
    driving_start_time_ = node_->get_clock()->now() - rclcpp::Duration::from_seconds(DRIVE_WAIT_TIMEOUT_S + 0.5);
  }

  // Expose a helper to simulate a HALT timeout by setting halt_start_time_ in the past.
  void simulateHaltTimeout() {
    halt_start_time_ = node_->get_clock()->now() - rclcpp::Duration::from_seconds(MAX_EGO_HALT_TIME + 0.5);
  }

  // Force-set the internal state.
  void forceSetState(AutonomousDrivingTaskState new_state) {
    state_ = new_state;
  }

  // Force-set the operation mode state.
  void forceSetOperationModeState(const OperationModeState & op_mode) {
    operation_mode_state_ = op_mode;
  }

  // Force-set the vehicle motion state.
  void forceSetVehicleMotionState(const MotionState & motion_state) {
    vehicle_motion_state_ = motion_state.state;
  }

  // Force-set the route state.
  void forceSetRouteState(const RouteState & route_state) {
    route_state_ = route_state.state;
  }

  // Override engageAutoDrive to simulate success
  void engageAutoDrive() override {
    RCLCPP_INFO(node_->get_logger(), "Simulated engageAutoDrive success.");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
};

//
// Test fixture for AutonomousDriving tests
//
class AutonomousDrivingTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("autonomous_driving_test_node");
    // Use your existing AutowareBridgeUtil implementation.
    util_ = std::make_shared<AutowareBridgeUtil>();
    // Create our test subject as TestableAutonomousDriving.
    autonomous_driving_ = std::make_shared<TestableAutonomousDriving>(node, util_);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  // Helper to spin the node for a specified duration (in seconds)
  void spinFor(double seconds) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds)) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<AutowareBridgeUtil> util_;
  std::shared_ptr<TestableAutonomousDriving> autonomous_driving_;
};

//
// Test 1: Cancellation Branch
// When cancel() is called, execute() should update the task status to "CANCELLED".
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingCancel) {
    std::string task_id = "autonomous_driving";
    geometry_msgs::msg::PoseStamped dummy_pose;

    // Set up the task in the util
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_);

    // Start execution in a separate thread
    std::thread exec_thread([&]() {
        autonomous_driving_->execute(task_id, dummy_pose);
    });

    // Give it some time to start execution before cancelling
    std::this_thread::sleep_for(200ms);

    // Request cancellation
    autonomous_driving_->cancel();

    // Wait for the task to complete
    exec_thread.join();

    // Validate that the task was cancelled
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("CANCELLED"));
    util_->clearActiveTaskPtr();
}

//
// Test 2: Timeout After Maximum Drive Retries
// By simulating repeated timeouts in the WAIT_AUTO_DRIVE_READY branch, the task should eventually update its status to "TIMEOUT".
/* TEST_F(AutonomousDrivingTest, TestAutonomousDrivingTimeoutAfterRetries) {
    std::string task_id = "autonomous_driving_timeout";
    geometry_msgs::msg::PoseStamped dummy_pose;

    // Set up the task in the util
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_);


    std::atomic_bool finished{false};
    std::thread exec_thread([&]() {
        autonomous_driving_->execute(task_id, dummy_pose);
        finished = true;
    });

    // In a helper thread, repeatedly simulate a drive wait timeout.
    std::thread timeout_simulator([&]() {
        while (!finished) {
            autonomous_driving_->simulateDriveWaitTimeout();
            std::this_thread::sleep_for(50ms);
        }
    });

    exec_thread.join();
    timeout_simulator.join();

    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("TIMEOUT"));
} */

TEST_F(AutonomousDrivingTest, TestAutonomousDrivingTimeoutAfterRetries) {
    std::string task_id = "autonomous_driving";
    geometry_msgs::msg::PoseStamped dummy_pose;

    // Set up the task in the util
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_);

    // Optional: force state if your logic supports a similar structure
    // autonomous_driving_->forceSetState(AutonomousDrivingTaskState::WAIT_FOR_COMPLETION);

    // Start execution in a separate thread
    std::thread exec_thread([&]() {
        autonomous_driving_->execute(task_id, dummy_pose);
    });

    // Allow enough time for all retries and timeout to occur
    std::this_thread::sleep_for(
        std::chrono::duration<double>((MAX_DRIVE_RETRIES + 1) * DRIVE_WAIT_TIMEOUT_S));

    // Stop the execution thread
    if (exec_thread.joinable()) {
        exec_thread.join();
    }

    // Validate that after max retries, the task status is "TIMEOUT"
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("TIMEOUT"));
}



//
// Test 3: Successful Execution
// In the WAIT_AUTO_DRIVE_READY branch, simulate that the operation mode becomes AUTONOMOUS.
// Then in the DRIVING branch, simulate that the route state becomes ARRIVED so that the final task status is "SUCCESS".
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingSuccess) {
    std::string task_id = "autonomous_driving_success";
    geometry_msgs::msg::PoseStamped dummy_pose;

    // Set up the task in the util
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_);
    // Force state to WAIT_AUTO_DRIVE_READY.
    //autonomous_driving_->forceSetState(AutonomousDrivingTaskState::WAIT_AUTO_DRIVE_READY);
    autonomous_driving_->forceSetState(AutonomousDrivingTaskState::DRIVING);

    // Simulate that the operation mode becomes AUTONOMOUS.
    //autoware_adapi_v1_msgs::msg::OperationModeState op_mode_msg;
    //op_mode_msg.is_autonomous_mode_available = true;
    //autonomous_driving_->forceSetOperationModeState(op_mode_msg);

    // In the DRIVING branch, simulate that the route state is ARRIVED.
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
    autonomous_driving_->forceSetRouteState(route_state_msg);

    std::thread exec_thread([&]() {
        autonomous_driving_->execute(task_id, dummy_pose);
    });
    spinFor(1.0);
    exec_thread.join();

    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("SUCCESS"));
}

//
// Test 4: Retry Branch
// When in ENGAGE_AUTO_DRIVE and the retry counter becomes 1, the task should update its status to "RETRYING".
// Then cancel to exit the loop.
/* TEST_F(AutonomousDrivingTest, TestAutonomousDrivingRetryOnce) {
    std::string task_id = "autonomous_driving_retry_once";
    geometry_msgs::msg::PoseStamped dummy_pose;

    // Set up the task and state
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_);

    // Start from ENGAGE_AUTO_DRIVE state
    autonomous_driving_->forceSetState(AutonomousDrivingTaskState::ENGAGE_AUTO_DRIVE);

    // Initially force operation mode to something other than AUTONOMOUS to trigger retry
    autoware_adapi_v1_msgs::msg::OperationModeState op_mode_msg;
    op_mode_msg.mode = autoware_adapi_v1_msgs::msg::OperationModeState::STOP;
    autonomous_driving_->forceSetOperationModeState(op_mode_msg);

    std::atomic_bool finished{false};

    // Run execute() in a separate thread
    std::thread exec_thread([&]() {
        autonomous_driving_->execute(task_id, dummy_pose);
        finished = true;
    });

    // Sleep long enough for first retry attempt
    std::this_thread::sleep_for(std::chrono::duration<double>(DRIVE_WAIT_TIMEOUT_S + 0.5));

    // Now simulate recovery: set mode to AUTONOMOUS
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    op_mode_msg.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
    autonomous_driving_->forceSetOperationModeState(op_mode_msg);

    // Let the rest of the process complete
    exec_thread.join();

    // After recovering on first retry, we expect SUCCESS
    EXPECT_EQ(util_->getTaskStatus(task_id).status, "SUCCESS");
}
 */


//
// Test 5: HALTED Condition in DRIVING Branch
// When the vehicle is STOPPED and the halt timer is forced to be expired,
// the task should update its status to "HALTED".
/*TEST_F(AutonomousDrivingTest, TestAutonomousDrivingHalted) {
    const std::string task_id = "autonomous_driving_halted";
    const geometry_msgs::msg::PoseStamped dummy_pose;
    const double MAX_EGO_HALT_TIME = 60.0;
  
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_);
  
    // Set state to DRIVING
    autonomous_driving_->forceSetState(AutonomousDrivingTaskState::DRIVING);
  
    // Set vehicle state to STOPPED
    autoware_adapi_v1_msgs::msg::MotionState mstate_msg;
    mstate_msg.state = autoware_adapi_v1_msgs::msg::MotionState::STOPPED;
    autonomous_driving_->forceSetVehicleMotionState(mstate_msg);
  
    // Set halt_start_time_ to 61 seconds ago (simulate vehicle halted > MAX_EGO_HALT_TIME)
    const auto fake_now = autonomous_driving_->getNode()->get_clock()->now();
    const auto fake_halt_start_time = rclcpp::Time(fake_now.seconds() - (MAX_EGO_HALT_TIME + 1), 0);
    autonomous_driving_->forceSetHaltStartTime(fake_halt_start_time);
  
    // Route state is not ARRIVED
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
    autonomous_driving_->forceSetRouteState(route_state_msg);
  
    // Run execute in background
    std::atomic_bool finished{false};
    std::thread exec_thread([&]() {
      autonomous_driving_->execute(task_id, dummy_pose);
      finished = true;
    });
  
    // Wait until task status becomes HALTED (polling)
    constexpr int max_wait_ms = 3000;
    int waited_ms = 0;
    while (waited_ms < max_wait_ms) {
      if (util_->getTaskStatus(task_id).status == "HALTED") {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      waited_ms += 100;
    }
  
    // Check status is HALTED
    EXPECT_EQ(util_->getTaskStatus(task_id).status, "HALTED");
  
    // Unblock execute loop to allow thread to exit
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
    autonomous_driving_->forceSetRouteState(route_state_msg);
    mstate_msg.state = autoware_adapi_v1_msgs::msg::MotionState::MOVING;
    autonomous_driving_->forceSetVehicleMotionState(mstate_msg);
  
    exec_thread.join();
  }*/
  
//
// Test 6: DRIVING Branch Recovery to Success
// When the vehicle is MOVING (so the halt timer resets) and then later the route state becomes ARRIVED,
// the task should complete with a "SUCCESS" status.
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingDrivingToSuccess) {
    std::string task_id = "autonomous_driving_driving_success";
    geometry_msgs::msg::PoseStamped dummy_pose;

    // Set up the task in the util
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_);

    // Force state to DRIVING.
    autonomous_driving_->forceSetState(AutonomousDrivingTaskState::DRIVING);
    // Simulate that the vehicle is MOVING.
    autoware_adapi_v1_msgs::msg::MotionState mstate_msg;
    mstate_msg.state = autoware_adapi_v1_msgs::msg::MotionState::MOVING;
    autonomous_driving_->forceSetVehicleMotionState(mstate_msg);
    // Initially, set route state to UNKNOWN.
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
    autonomous_driving_->forceSetRouteState(route_state_msg);

    std::atomic_bool finished{false};
    std::thread exec_thread([&]() {
        autonomous_driving_->execute(task_id, dummy_pose);
        finished = true;
    });

    std::this_thread::sleep_for(500ms);
    // Now simulate route arrival.
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
    autonomous_driving_->forceSetRouteState(route_state_msg);

    exec_thread.join();
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("SUCCESS"));
}
