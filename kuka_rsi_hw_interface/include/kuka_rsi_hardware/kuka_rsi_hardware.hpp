// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef KUKA_RSI_HARDWARE__KUKA_RSI_HARDWARE_HPP_
#define KUKA_RSI_HARDWARE__KUKA_RSI_HARDWARE_HPP_

#include <memory>
#include <string>

#include "angles/angles.h"

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// #include "kuka_rsi_hardware/rsi_command.h"
// #include "kuka_rsi_hardware/rsi_state.h"
#include "kuka_rsi_hardware/udp_server.h"
#include "kuka_rsi_hardware/visibility_control.h"

namespace kuka_rsi_hardware
{

using namespace angles;

class KukaRsiHardware : public hardware_interface::RobotHardware
{
public:
    KUKA_RSI_HARDWARE_PUBLIC
    hardware_interface::hardware_interface_ret_t init();

    KUKA_RSI_HARDWARE_PUBLIC
    hardware_interface::hardware_interface_ret_t read();

    KUKA_RSI_HARDWARE_PUBLIC
    hardware_interface::hardware_interface_ret_t write();

    std::array<std::string, 6> joint_names_ = {
        "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};
    std::array<double, 6> joint_position_ = {0.0, from_degrees(-90.0), from_degrees(90.0), 0.0, from_degrees(90.0), 0.0};
    std::array<double, 6> joint_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> joint_effort_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> joint_position_command_ = {0.0, 0.0, 0.0,
                                                     0.0, 0.0, 0.0};

    std::array<bool, 6> read_op_ = {false, false, false, false, false, false};
    std::array<bool, 6> write_op_ = {true, true, true, true, true, true};

    std::array<std::string, 6> read_op_handle_names_ = {
        "read1", "read2", "read3",
        "read4", "read5", "read6"};

    std::array<std::string, 6> write_op_handle_names_ = {
        "write1", "write2", "write3",
        "write4", "write5", "write6"};

    std::array<hardware_interface::JointStateHandle, 6> joint_state_handles_;
    std::array<hardware_interface::JointCommandHandle, 6> joint_command_handles_;
    std::array<hardware_interface::OperationModeHandle, 6> read_op_handles_;
    std::array<hardware_interface::OperationModeHandle, 6> write_op_handles_;

    std::unique_ptr<UDPServer> server_ = nullptr;
};

} // namespace kuka_rsi_hardware
#endif // KUKA_RSI_HARDWARE__KUKA_RSI_HARDWARE_HPP_
