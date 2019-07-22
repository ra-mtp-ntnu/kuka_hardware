#include "follow_joint_trajectory_action.hpp"

FollowJointTrajectoryAction::FollowJointTrajectoryAction(const rclcpp::NodeOptions &options)
    : Node("follow_joint_trajectory_action_server", options)
{
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "follow_joint_trajectory",
      std::bind(&FollowJointTrajectoryAction::handle_goal, this, _1, _2),
      std::bind(&FollowJointTrajectoryAction::handle_cancel, this, _1),
      std::bind(&FollowJointTrajectoryAction::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse FollowJointTrajectoryAction::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;

  // for(int i = 0; i <  goal->trajectory.points.size()
  //   for(int j = 0; j <  goal->trajectory.points[i].positions.size(); j++) {
  //     printf("position:\t%.5f\t",  goal->trajectory.points[i].positions[j]);
  //   }
  //   printf("\n");
  //   for(int j = 0; j <  goal->trajectory.points[i].velocities.size(); j++) {
  //     printf("velocities:\t%.5f\t",  goal->trajectory.points[i].velocities[j]);
  //   }
  //   printf("\n");
  //   printf("time_from_start %u %u\n", goal->trajectory.points[i].time_from_start.sec, goal->trajectory.points[i].time_from_start.nanosec);
  // }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowJointTrajectoryAction::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowJointTrajectoryAction::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  /* 
  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  // wait for all action servers
  bool all_ready = true;
  for (size_t i = 0; i < action_clients.size(); ++i)
  {
    bool ready = action_clients.at(i)->wait(5);
    if (ready)
    {
      RCLCPP_INFO(this->get_logger(), "%s: ok", action_clients.at(i)->getName().c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "%s: failed", action_clients.at(i)->getName().c_str());
    }

    all_ready &= ready;
  }
  if (!all_ready)
  {
    RCLCPP_ERROR(this->get_logger(), "Not all action server ready. Exit.");
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();

  //send goals
  bool all_succeed = true;

  // Populate a goal
  std::vector<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal> vector_goal;
  for (int i = 0; i < action_clients.size(); i++)
  {
    vector_goal.push_back(hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal());
  }

  for (unsigned int v = 0; v < vector_goal.size(); v++)
  {
    vector_goal[v].trajectory.points.resize(goal->trajectory.points.size());
    for (unsigned int i = 0; i < goal->trajectory.points.size(); i++)
    {
      vector_goal[v].trajectory.points[i].positions.resize(1);
      vector_goal[v].trajectory.points[i].velocities.resize(1);
      vector_goal[v].trajectory.points[i].accelerations.resize(1);

      vector_goal[v].trajectory.points[i].positions[0] = goal->trajectory.points[i].positions[v];
      vector_goal[v].trajectory.points[i].velocities[0] = goal->trajectory.points[i].velocities[v];
      vector_goal[v].trajectory.points[i].accelerations[0] = goal->trajectory.points[i].accelerations[v];
      vector_goal[v].trajectory.points[i].time_from_start.sec = goal->trajectory.points[i].time_from_start.sec;
      vector_goal[v].trajectory.points[i].time_from_start.nanosec = goal->trajectory.points[i].time_from_start.nanosec;
    }

    double wait_time = (double)(vector_goal[v].trajectory.points[goal->trajectory.points.size() - 1].time_from_start.sec) +
                       (double)(vector_goal[v].trajectory.points[goal->trajectory.points.size() - 1].time_from_start.nanosec / 1e+9) + 1.0;
  }

  for (unsigned int v = 0; v < vector_goal.size(); v++)
  {
    all_succeed &= action_clients.at(v)->send_goal(vector_goal[v]);
  }

  if (!all_succeed)
  {
    RCLCPP_ERROR(this->get_logger(), "Not all action commands succeeded. Exit.");
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "All goal commands sent");

  RCLCPP_INFO(this->get_logger(), "Executing goal");

  while (true)
  {
    all_succeed = true;
    for (unsigned int v = 0; v < vector_goal.size(); v++)
    {
      all_succeed &= action_clients.at(v)->is_goal_done();
    }
    if (all_succeed)
      break;
  }

  // // Check if goal is done
  if (rclcpp::ok())
  {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Suceeded");
  }
  */
}

void FollowJointTrajectoryAction::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  using namespace std::placeholders;
  RCLCPP_INFO(this->get_logger(), "handle_accepted and executing");

  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&FollowJointTrajectoryAction::execute, this, _1), goal_handle}.detach();
}
