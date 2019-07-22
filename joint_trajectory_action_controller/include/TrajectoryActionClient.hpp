#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hrim_actuator_rotaryservo_actions/action/goal_joint_trajectory.hpp"
#include <thread>

#include  <cmath>
#include  <cstdlib>

static rclcpp::Logger LOGGER = rclcpp::get_logger("TrajectoryActionClient");

class TrajectoryActionClient
{
public:
  using GoalJointTrajectory = hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory;
  using GoalHandleJointTrajectory = rclcpp_action::ClientGoalHandle<GoalJointTrajectory>;

  TrajectoryActionClient(  rclcpp_action::Client<GoalJointTrajectory>::SharedPtr client_ptr, std::string action_name)
  : goal_done_(false), status_(0), action_name_(action_name),
    following_error_(0.0)
  {
    RCLCPP_INFO(LOGGER,  "TrajectoryActionClient: %s ", action_name.c_str() );
    this->client_ptr_ = client_ptr;
  }

  uint get_status() const
  {
    return this->status_;
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  bool cancel() const
  {
    auto cancel_result_future = this->client_ptr_->async_cancel_goal(goal_handle_);
    return true;
  }

  double get_following_error()
  {
    return following_error_;
  }

  bool wait(long int timeout)
  {
    if (!this->client_ptr_) {
      RCLCPP_ERROR(LOGGER,  "Action client not initialized");
      return false;
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(timeout))) {
      //RCLCPP_ERROR(LOGGER,  "Action server not available after waiting");
      return false;
    }

    return true;
  }

  bool send_goal(hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal goal_msg)
  {
    using namespace std::placeholders;

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(LOGGER,  "Action client not initialized");
    }

    RCLCPP_INFO(LOGGER,  "%s: Sending goal",action_name_.c_str() );

    using namespace std::placeholders;
    auto send_goal_options = rclcpp_action::Client<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&TrajectoryActionClient::result_callback, this, _1);
    // send_goal_options.goal_response_callback = std::bind(&FollowJointTrajectoryControllerHandle::controllerActiveCallback, this, _1);
    send_goal_options.feedback_callback =  std::bind(&TrajectoryActionClient::feedback_callback, this, _1, _2);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    // // check call result
    // if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    //   rclcpp::executor::FutureReturnCode::SUCCESS)
    // {
    //   RCLCPP_ERROR(LOGGER,  "send goal call failed :(");
    //   this->goal_done_ = true;
    //   return false;
    // }

    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(LOGGER,  "%s: Goal was rejected by server", action_name_.c_str());
      this->goal_done_ = true;
      return false;
    }

    bool action_server_is_ready = this->client_ptr_->action_server_is_ready();

    return true;
  }

  std::string getName()
  {
    return action_name_;
  }

private:
  rclcpp_action::Client<GoalJointTrajectory>::SharedPtr client_ptr_;
  bool goal_done_;
  std::string action_name_;
  rclcpp_action::ClientGoalHandle<GoalJointTrajectory>::SharedPtr goal_handle_;
  uint status_;
  double following_error_;

  void goal_response_callback(std::shared_future<GoalHandleJointTrajectory::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER,  "Goal was rejected by server");
    } else {
      RCLCPP_INFO(LOGGER,  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleJointTrajectory::SharedPtr,
    const std::shared_ptr<const GoalJointTrajectory::Feedback> feedback)
  {
      double time_from_start = (double)(feedback->actual.time_from_start.sec) + (double)(feedback->actual.time_from_start.nanosec/1e+9);
      double time_from_start_desired = (double)(feedback->desired.time_from_start.sec) + (double)(feedback->desired.time_from_start.nanosec/1e+9);
      double time_from_start_error = (double)(feedback->error.time_from_start.sec) + (double)(feedback->error.time_from_start.nanosec/1e+9);

      // TODO
      // following_error_ = feedback->error.positions[0];
      // if(feedback->error.positions[0] > 0.2){
      //   //error !!
      // }
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::WrappedResult & result)
  {
    this->goal_done_ = true;
    // switch (result.code) {
    //   case rclcpp_action::ResultCode::SUCCEEDED:
    //     break;
    //   case rclcpp_action::ResultCode::ABORTED:
    //     RCLCPP_ERROR(LOGGER,  "Goal was aborted");
    //     return;
    //   case rclcpp_action::ResultCode::CANCELED:
    //     RCLCPP_ERROR(LOGGER,  "Goal was canceled");
    //     return;
    //   default:
    //     RCLCPP_ERROR(LOGGER,  "Unknown result code");
    //     return;
    // }
  }
};  // class TrajectoryActionClient
