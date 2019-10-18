#include <memory>

#include "controller_manager/controller_manager.hpp"

#include "kuka_rsi_hardware/kuka_rsi_hardware.hpp"

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // create my_robot instance
  auto my_robot = std::make_shared<kuka_rsi_hardware::KukaRsiHardware>();

  // initialize the robot
  if (my_robot->init() != hardware_interface::HW_RET_OK)
  {
    fprintf(stderr, "failed to initialize hardware\n");
    return -1;
  }

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // start the controller manager with the robot hardware
  controller_manager::ControllerManager cm(my_robot, executor);

  // load the joint state controller.
  cm.load_controller("ros_controllers", "ros_controllers::JointStateController", "joint_state_controller");

  // load the trajectory controller
  auto etasl_ros_controller = cm.load_controller(
      "etasl_ros2_controllers", "etasl_ros2_controllers::EtaslRos2Controller", "etasl_ros2_controller");

  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6" };
  rclcpp::Parameter joint_parameters("joints", joint_names);
  etasl_ros_controller->get_lifecycle_node()->set_parameter(joint_parameters);

  std::vector<std::string> operation_mode_names = { "write1", "write2", "write3", "write4", "write5", "write6" };
  rclcpp::Parameter operation_mode_parameters("write_op_modes", operation_mode_names);
  etasl_ros_controller->get_lifecycle_node()->set_parameter(operation_mode_parameters);

  std::string task_specification = "/home/lars/etasl_ros2_control_ws/src/etasl_ros2_control/"
                                   "etasl_ros2_control_examples/scripts/example_kuka_2.lua";
  rclcpp::Parameter task_specification_parameter("task_specification", task_specification);
  etasl_ros_controller->get_lifecycle_node()->set_parameter(task_specification_parameter);

  std::vector<std::string> input_names = { "tgt1", "tgt2", "vec", "rotation", "frame", "twist", "wrench" };
  rclcpp::Parameter input_name_parameters("input_names", input_names);
  etasl_ros_controller->get_lifecycle_node()->set_parameter(input_name_parameters);

  std::vector<std::string> input_types = { "Scalar", "Scalar", "Vector", "Rotation", "Frame", "Twist", "Wrench" };
  rclcpp::Parameter input_type_parameters("input_types", input_types);
  etasl_ros_controller->get_lifecycle_node()->set_parameter(input_type_parameters);

  // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
  auto future_handle = std::async(std::launch::async, spin, executor);

  // we can either configure each controller individually through its services
  // or we use the controller manager to configure every loaded controller
  if (cm.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
    RCLCPP_ERROR(cm.get_logger(), "at least one controller failed to configure");
    return -1;
  }
  // and activate all controller
  if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
    RCLCPP_ERROR(cm.get_logger(), "at least one controller failed to activate");
    return -1;
  }

  rclcpp::Rate r(100);
  // main loop
  hardware_interface::hardware_interface_ret_t ret;
  while (rclcpp::ok())
  {
    ret = my_robot->read();

    if (ret != hardware_interface::HW_RET_OK)
    {
      fprintf(stderr, "read failed!\n");
    }

    cm.update();

    ret = my_robot->write();
    if (ret != hardware_interface::HW_RET_OK)
    {
      fprintf(stderr, "write failed!\n");
    }

    r.sleep();
  }

  executor->cancel();
}