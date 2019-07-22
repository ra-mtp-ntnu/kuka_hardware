#include <rclcpp/rclcpp.hpp>
#include "follow_joint_trajectory_action.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<FollowJointTrajectoryAction>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
