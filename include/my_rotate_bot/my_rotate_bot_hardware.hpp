#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
// #include "ros2_control_demo_hardware/visibility_control.h"

using hardware_interface::return_type;

namespace my_rotate_bot_namespace
{
class MyRotateBotHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyRotateBotHardware)

//   ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

//   ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

//   ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type start() override;

//   ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type stop() override;

//   ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type read() override;

//   ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type write() override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
