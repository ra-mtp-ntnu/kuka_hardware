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

/* Original Author: Lars Tingelstad
 * Co-Author: Mathias Hauan Arbo
 */

#ifndef KUKA_RSI_HARDWARE__KUKA_RSI_HARDWARE_HPP_
#define KUKA_RSI_HARDWARE__KUKA_RSI_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "angles/angles.h"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "kuka_rsi_hardware/visibility_control.h"

#include "kuka_rsi_hardware/rsi_command.h"
#include "kuka_rsi_hardware/rsi_state.h"
#include "kuka_rsi_hardware/udp_server.h"

using hardware_interface::return_type;

namespace kuka_rsi_hardware
{

using namespace angles;

class KukaRSIHardware : public 
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRSIHardware);

  KUKA_RSI_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  KUKA_RSI_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_RSI_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KUKA_RSI_HARDWARE_PUBLIC
  return_type start() override;

  KUKA_RSI_HARDWARE_PUBLIC
  return_type stop() override;

  KUKA_RSI_HARDWARE_PUBLIC
  return_type read() override;

  KUKA_RSI_HARDWARE_PUBLIC
  return_type write() override;

private:
  std::vector<double> joint_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> joint_position_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // RSI 
  RSIState rsi_state_;
  RSICommand rsi_command_;
  unsigned long long ipoc_;
  std::vector<double> rsi_initial_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> rsi_joint_positions_corrections_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Network connection
  std::unique_ptr<UDPServer> server_ = nullptr;
  std::string rsi_ip_;
  int rsi_port_;
  std::string in_buffer_;
  std::string out_buffer_;

};

} // namespace kuka_rsi_hardware
#endif // KUKA_RSI_HARDWARE__KUKA_RSI_HARDWARE_HPP_
