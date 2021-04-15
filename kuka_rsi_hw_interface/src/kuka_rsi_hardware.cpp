// Copyright 2021 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Lars Tingelstad
 * Co-Author: Mathias Hauan Arbo
 */

#include "kuka_rsi_hardware/kuka_rsi_hardware.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kuka_rsi_hardware
{

return_type KukaRSIHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }
  // Configure joints
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // KUKA RSI Hardware interface has only position interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaRSIHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaRSIHardware"),
        "Joint '%s' has '%s' command interface found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaRSIHardware"),
        "Joint '%s' has %d state interfaces found. 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaRSIHardware"),
        "Joint '%s' has '%s' state interface found. '%s' expected",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
  }
  // Prepare network communication
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);

  // Gather RSI parameters
  rsi_ip_ = info_.hardware_parameters["rsi_ip"];
  rsi_port_ = stoi(info_.hardware_parameters["rsi_port"]);
  RCLCPP_INFO(
    rclcpp::get_logger("KukaRSIHardware"),
    "Completed configure!");
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
KukaRSIHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_position_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaRSIHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_position_command_[i]));
  }
  return command_interfaces;
}

return_type KukaRSIHardware::start()
{
  server_.reset(new UDPServer(rsi_ip_, rsi_port_));
  RCLCPP_INFO(
    rclcpp::get_logger("KukaRSIHardware"),
    "Waiting for RSI connection!");
  
  int bytes = server_->recv(in_buffer_);
  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100) {
    bytes = server_->recv(in_buffer_);
  }
  rsi_state_ = RSIState(in_buffer_);
  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_position_[i] = from_degrees(rsi_state_.positions[i]);
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = from_degrees(rsi_state_.initial_positions[i]);
  }
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(rsi_joint_positions_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  RCLCPP_INFO(
    rclcpp::get_logger("KukaRSIHardware"),
    "Got RSI connection!");
  
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type KukaRSIHardware::stop()
{
  server_.reset();
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type KukaRSIHardware::read()
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0) {
    return return_type::ERROR;
  }

  rsi_state_ = RSIState(in_buffer_);
  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_position_[i] = from_degrees(rsi_state_.positions[i]);
  }
  ipoc_ = rsi_state_.ipoc;
  return return_type::OK;
}

return_type KukaRSIHardware::write()
{
  out_buffer_.resize(1024);
  for (uint i = 0; i < info_.joints.size(); i++) {
    rsi_joint_positions_corrections_[i] = to_degrees(joint_position_command_[i]) - to_degrees(rsi_initial_joint_positions_[i]);
  }

  out_buffer_ = RSICommand(rsi_joint_positions_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);

  return return_type::OK;
}

}  // namespace kuka_rsi_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kuka_rsi_hardware::KukaRSIHardware, hardware_interface::SystemInterface)
