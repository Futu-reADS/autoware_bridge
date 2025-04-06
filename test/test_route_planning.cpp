#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// Apply macro hack ONLY when including route_planning.hpp
#define private public
#define protected public
#include "autoware_bridge/route_planning.hpp"
#undef private
#undef protected

//enum class RouteState {UNKNOWN,UNSET,SET,ARRIVED,CHANGING};
// TestableRoutePlanning: Subclass of RoutePlanning that exposes private methods
class TestableRoutePlanning : public RoutePlanning {
public:
    using RoutePlanning::RoutePlanning;  // Inherit constructors

    // Expose private methods for testing
    // Expose private helper to publish target pose
    void testPublishTargetPose(const geometry_msgs::msg::PoseStamped &goal_pose) {
        publishTargetPose(goal_pose);
    }

    // Expose the private callbacks
    void testRouteStateCallback(const autoware_adapi_v1_msgs::msg::RouteState &msg) {
        routeStateCallback(msg);
    }
    void testOperationModeStateCallback(const autoware_adapi_v1_msgs::msg::OperationModeState &msg) {
        operationModeStateCallback(msg);
    }

    // Expose getters for private members
    RoutePlanningTaskState getTaskState() const {
        return state_;
    }
    uint16_t getRouteState() const {
        return route_state_;
    }
    autoware_adapi_v1_msgs::msg::OperationModeState getOperationModeState() const {
        return operation_mode_state_;
    }

    // Force setters for private members
    void forceSetState(RoutePlanningTaskState new_state) {
        state_ = new_state;
    }
    void forceSetRouteState(const autoware_adapi_v1_msgs::msg::RouteState &route_state_msg) {
        // route_state_ is a uint16_t; we set it from the message's state field.
        route_state_ = route_state_msg.state;
    }

    void forceSetOperationModeState(const autoware_adapi_v1_msgs::msg::OperationModeState &op_mode_msg) {
        operation_mode_state_ = op_mode_msg;
    }
    // Helper to simulate a planning timeout by setting planning start time far in the past.
    void simulatePlanningTimeout() {
        route_planning_start_time_ = node_->get_clock()->now() - rclcpp::Duration::from_seconds(ROUTE_PLANNING_TIMEOUT_S + 1);
    }
    
};

class RoutePlanningTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("route_planning_test");
        util_ = std::make_shared<AutowareBridgeUtil>();
        route_planning_ = std::make_shared<TestableRoutePlanning>(node, util_);
    }

    void TearDown() override {
        //util_->clearActiveTaskPtr();
        resetRoutePlanningTask();
        rclcpp::shutdown();
    }

    void resetRoutePlanningTask() {
        route_planning_->forceSetState(RoutePlanningTaskState::SET_GOAL);
        autoware_adapi_v1_msgs::msg::RouteState state_msg;
        state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
        route_planning_->forceSetRouteState(state_msg);
        util_->clearActiveTaskPtr();
    }
    // Helper function to create a dummy pose
    geometry_msgs::msg::PoseStamped createDummyPose() {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = node->now();
        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = 1.0;
        goal_pose.pose.position.y = 1.0;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;
        return goal_pose;
    }

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<AutowareBridgeUtil> util_;  
    std::shared_ptr<TestableRoutePlanning> route_planning_;
};

TEST_F(RoutePlanningTest, TestRoutePlanningSuccess) {
    std::string task_id = "route_planning";

    geometry_msgs::msg::PoseStamped goal_pose = createDummyPose();
    // Set up the task in the util
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);

    route_planning_->forceSetState(RoutePlanningTaskState::WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE);
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
    route_planning_->forceSetRouteState(route_state_msg);
    
    autoware_adapi_v1_msgs::msg::OperationModeState op_mode_msg;
    op_mode_msg.is_autonomous_mode_available = true;
    route_planning_->forceSetOperationModeState(op_mode_msg);

    

    // Execute in a separate thread to simulate async behavior
    std::thread exec_thread([&]() {
        route_planning_->execute(task_id, goal_pose);
    });

    exec_thread.join();  // Ensure the thread has finished execution

    // Validate the result
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("SUCCESS"));
}

  TEST_F(RoutePlanningTest, TestRoutePlanningCancel) {
    std::string task_id = "route_planning";
    geometry_msgs::msg::PoseStamped goal_pose = createDummyPose();

    // Set up the task
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);

    // Start execution in a separate thread
    std::thread exec_thread([&]() {
        route_planning_->execute(task_id, goal_pose);
    });

    // Give it some time to start execution before cancelling
    std::this_thread::sleep_for(200ms);

    // Request cancellation
    route_planning_->cancel();

    // Wait for the task to complete
    exec_thread.join();

    // Validate that the task was cancelled
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("CANCELLED"));
    //util_->clearActiveTaskPtr();
}

TEST_F(RoutePlanningTest, TestRoutePlanningTimeoutAfterRetries) {
    std::string task_id = "route_planning";
    geometry_msgs::msg::PoseStamped goal_pose = createDummyPose();

    // Set up the task in the util
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);

    // Force the state to SET_GOAL so that we begin in the correct branch.
    route_planning_->forceSetState(RoutePlanningTaskState::SET_GOAL);

    // Execute the task in a separate thread because execute() is blocking.
    std::thread exec_thread([&]() {
        route_planning_->execute(task_id, goal_pose);
    });

    // Allow enough time for retries and timeout to occur.
    std::this_thread::sleep_for(std::chrono::duration<double>((MAX_ROUTE_PLANNING_RETRIES + 1) * ROUTE_PLANNING_TIMEOUT_S));


    // Stop the execution thread.
    if (exec_thread.joinable()) {
        exec_thread.join();
    }

    // Validate that after max retries, the task status is "TIMEOUT"
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("TIMEOUT"));
    //util_->clearActiveTaskPtr();
}
 
 TEST_F(RoutePlanningTest, TestOperationModeNotActiveRetry) {
    std::string task_id = "route_planning_mode_inactive";
    auto goal_pose = createDummyPose();
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
    route_planning_->forceSetRouteState(route_state_msg);

    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);

    // Force state to WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE.
    route_planning_->forceSetState(RoutePlanningTaskState::WAIT_FOR_AUTOWARE_TO_ENABLE_AUTO_MODE);
    // Simulate that autonomous mode is NOT available.
    autoware_adapi_v1_msgs::msg::OperationModeState op_mode_msg;
    op_mode_msg.is_autonomous_mode_available = false;
    route_planning_->forceSetOperationModeState(op_mode_msg);
    // Simulate that more than 10 seconds have elapsed by setting route_state_SET_start_time_ accordingly.
    route_planning_->route_state_SET_start_time_ = node->get_clock()->now() - rclcpp::Duration::from_seconds(10.1);

    std::thread exec_thread([&]() {
        route_planning_->execute(task_id, goal_pose);
    });
    // Wait briefly, then cancel.
    std::this_thread::sleep_for(500ms);
    // Log current state before cancel
    RCLCPP_INFO(node->get_logger(), "Task state before cancel: %d", static_cast<int>(route_planning_->getTaskState()));

    route_planning_->cancel();

    RCLCPP_INFO(node->get_logger(), "Task state after cancel: %d", static_cast<int>(route_planning_->getTaskState()));

    exec_thread.join();
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("CANCELLED"));
    //util_->clearActiveTaskPtr();
}


TEST_F(RoutePlanningTest, TestRoutePlanningFailure) {
    std::string task_id = "task_4";
    geometry_msgs::msg::PoseStamped goal_pose = createDummyPose();

    // Force a RouteState that would lead to failure (e.g., already ARRIVED).
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
    route_planning_->forceSetRouteState(route_state_msg);

    // Set up the task utilities
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);

    // Execute task in the current (failing) state
    route_planning_->execute(task_id, goal_pose);

    // Expect the task to report failure
    EXPECT_EQ(util_->getTaskStatus(task_id).status, "FAILED");
    //util_->clearActiveTaskPtr();
}

TEST_F(RoutePlanningTest, TestRouteStateUnknown) {
    std::string task_id = "task_unknown";
    geometry_msgs::msg::PoseStamped goal_pose = createDummyPose();
  
    // Set up the task.
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);
  
    // Force the task to start from the SET_GOAL branch.
    route_planning_->forceSetState(RoutePlanningTaskState::SET_GOAL);
  
    // Force the route state to UNKNOWN.
    autoware_adapi_v1_msgs::msg::RouteState unknown_state;
    unknown_state.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
    route_planning_->forceSetRouteState(unknown_state);
  
    // Create an atomic flag to signal when execute() has finished.
    std::atomic_bool finished{false};
  
    // Start execute() in a separate thread.
    std::thread exec_thread([&]() {
      route_planning_->execute(task_id, goal_pose);
      finished = true;
    });
  
    // Helper thread to repeatedly simulate a timeout by adjusting the planning start time.
    // This forces the condition in WAIT_FOR_AUTOWARE_ROUTE_PLANNING to be true even after the state is reset.
    std::thread timeout_simulator([&]() {
      while (!finished) {
        route_planning_->simulatePlanningTimeout();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  
    // Wait for the execution thread to finish.
    exec_thread.join();
    timeout_simulator.join();
  
    // The expected behavior is that after exhausting retries, the task status becomes "TIMEOUT".
    EXPECT_EQ(util_->getTaskStatus(task_id).status, "TIMEOUT");
    //util_->clearActiveTaskPtr();
  }
  

  TEST_F(RoutePlanningTest, TestRouteStateChanging) {
    std::string task_id = "task_changing";
    auto goal_pose = createDummyPose();
  
    // Set up the task.
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);
  
    // Begin from the SET_GOAL branch.
    route_planning_->forceSetState(RoutePlanningTaskState::SET_GOAL);
  
    // Force the route state to CHANGING.
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::CHANGING;
    route_planning_->forceSetRouteState(route_state_msg);
  
    // Atomic flag to track when execution is complete.
    std::atomic_bool finished{false};
  
    // Run execute() in a separate thread.
    std::thread exec_thread([&]() {
      route_planning_->execute(task_id, goal_pose);
      finished = true;
    });
  
    // Continuously simulate timeout while execute is running.
    std::thread timeout_simulator([&]() {
      while (!finished) {
        route_planning_->simulatePlanningTimeout();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  
    // Wait for the threads to finish.
    exec_thread.join();
    timeout_simulator.join();
  
    // Verify that the task eventually times out due to stuck CHANGING state.
    EXPECT_EQ(util_->getTaskStatus(task_id).status, "TIMEOUT");
  
    // Clean up if needed
    // util_->clearActiveTaskPtr();
  }
  

 TEST_F(RoutePlanningTest, TestRouteStateUnset) {
  std::string task_id = "task_unset";
  auto goal_pose = createDummyPose();

  // Set up the task.
  util_->updateTaskId(task_id);
  util_->setActiveTaskPtr(route_planning_);

  // Begin from the SET_GOAL branch.
  route_planning_->forceSetState(RoutePlanningTaskState::SET_GOAL);

  // Force the route state to UNSET.
  autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
  route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::UNSET;
  route_planning_->forceSetRouteState(route_state_msg);

  // Atomic flag to signal when execute() has finished.
  std::atomic_bool finished{false};

  // Start execute() in a separate thread.
  std::thread exec_thread([&]() {
    route_planning_->execute(task_id, goal_pose);
    finished = true;
  });

  // Helper thread to repeatedly simulate planning timeout.
  // This ensures we hit the timeout logic even with retries.
  std::thread timeout_simulator([&]() {
    while (!finished) {
      route_planning_->simulatePlanningTimeout();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // Wait for the execution thread to finish.
  exec_thread.join();
  timeout_simulator.join();

  // Expect that after retries are exhausted, the task status is "TIMEOUT".
  EXPECT_EQ(util_->getTaskStatus(task_id).status, "TIMEOUT");

  // Clean up
  //util_->clearActiveTaskPtr();
}  
  
TEST_F(RoutePlanningTest, TestRoutePlanningRetryOnce) {
    std::string task_id = "task_retry_once";
    geometry_msgs::msg::PoseStamped goal_pose = createDummyPose();
  
    // Set up the task and state
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);
  
    // Start from SET_GOAL state
    route_planning_->forceSetState(RoutePlanningTaskState::SET_GOAL);
  
    // Force route state to UNKNOWN (which should trigger retries)
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
    route_planning_->forceSetRouteState(route_state_msg);
  
    // Track execution status
    std::atomic_bool finished{false};
  
    // Run execute() in a separate thread
    std::thread exec_thread([&]() {
      route_planning_->execute(task_id, goal_pose);
      finished = true;
    });
  
    // Simulate a timeout once to trigger retry
    std::this_thread::sleep_for(std::chrono::duration<double>(ROUTE_PLANNING_TIMEOUT_S + 0.5));
    route_planning_->simulatePlanningTimeout();
  
    // Before second timeout, set route state to SET to simulate recovery
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
    route_planning_->forceSetRouteState(route_state_msg);
  
    // Also simulate AUTO mode becoming available
    autoware_adapi_v1_msgs::msg::OperationModeState op_mode_msg;
    op_mode_msg.is_autonomous_mode_available = true;
    route_planning_->forceSetOperationModeState(op_mode_msg);
  
    // Let the rest of the process complete
    exec_thread.join();
  
    // After recovering on first retry, we expect SUCCESS
    EXPECT_EQ(util_->getTaskStatus(task_id).status, "SUCCESS");
  }
  
  TEST_F(RoutePlanningTest, TestRoutePlanningRetrying) {
    std::string task_id = "task_retrying";
    geometry_msgs::msg::PoseStamped goal_pose = createDummyPose();

    // Set up the task
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(route_planning_);

    // Force the state to SET_GOAL
    route_planning_->forceSetState(RoutePlanningTaskState::SET_GOAL);

    // Force route state to UNKNOWN to simulate initial failure
    autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
    route_planning_->forceSetRouteState(route_state_msg);

    // Track execution status
    std::atomic_bool finished{false};

    // Run execute() in a separate thread
    std::thread exec_thread([&]() {
        route_planning_->execute(task_id, goal_pose);
        finished = true;
    });

    // Simulate a timeout to trigger retry
    std::this_thread::sleep_for(std::chrono::duration<double>(ROUTE_PLANNING_TIMEOUT_S / 2)); // Simulate half timeout
    route_planning_->simulatePlanningTimeout(); // First timeout, should trigger retry

    // Ensure that the state is RETRYING before continuing
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Give it some time to retry
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("RETRYING"));

    // Now allow the task to complete after retry
    // Simulate the second part of the process by setting the route state to SET
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    route_state_msg.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
    route_planning_->forceSetRouteState(route_state_msg);

    // Simulate AUTO mode being available
    autoware_adapi_v1_msgs::msg::OperationModeState op_mode_msg;
    op_mode_msg.is_autonomous_mode_available = true;
    route_planning_->forceSetOperationModeState(op_mode_msg);

    // Let the execution thread finish
    exec_thread.join();

    // Validate that after retrying, the task completes successfully
    EXPECT_EQ(util_->getTaskStatus(task_id).status, std::string("SUCCESS"));
}



 /*uint16 UNKNOWN = 0, UNSET = 1, SET = 2, ARRIVED = 3, CHANGING = 4
  ********************************************************* 
  uint16_t route_state_;
*/
/* uint16 UNKNOWN = 0, STOPPED = 1, STARTING = 2, MOVING = 3
  ********************************************************* 
  uint16_t vehicle_motion_state_;
*/
/* uint16 UNKNOWN = 0, UNINITIALIZED = 1, INITIALIZING = 2, INITIALIZED = 3
  ********************************************************* 
  uint16_t localization_state_;
*/
/*uint16 UNKNOWN = 0, STOP = 1, AUTONOMOUS = 2, LOCAL = 3, REMOTE = 4
  bool is_autonomous_mode_available
  *********************************************************
  OperationModeState operation_mode_state_;*/
