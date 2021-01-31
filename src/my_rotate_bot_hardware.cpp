#include "my_rotate_bot/my_rotate_bot_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_rotate_bot_namespace
{

return_type MyRotateBotHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

//   for (const hardware_interface::ComponentInfo & joint : info_.joints) {
//     // RRBotSystemPositionOnly has exactly one state and command interface on each joint
//     if (joint.command_interfaces.size() != 1) {
//       RCLCPP_FATAL(
//         rclcpp::get_logger("MyRotateBotHardware"),
//         "Joint '%s' has %d command interfaces found. 1 expected.",
//         joint.name.c_str(), joint.command_interfaces.size());
//       return return_type::ERROR;
//     }

//     if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
//       RCLCPP_FATAL(
//         rclcpp::get_logger("MyRotateBotHardware"),
//         "Joint '%s' have %s command interfaces found. '%s' expected.",
//         joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
//         hardware_interface::HW_IF_POSITION);
//       return return_type::ERROR;
//     }

//     if (joint.state_interfaces.size() != 1) {
//       RCLCPP_FATAL(
//         rclcpp::get_logger("MyRotateBotHardware"),
//         "Joint '%s' has %d state interface. 1 expected.",
//         joint.name.c_str(), joint.state_interfaces.size());
//       return return_type::ERROR;
//     }

//     if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
//       RCLCPP_FATAL(
//         rclcpp::get_logger("MyRotateBotHardware"),
//         "Joint '%s' have %s state interface. '%s' expected.",
//         joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
//         hardware_interface::HW_IF_POSITION);
//       return return_type::ERROR;
//     }
//   }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
MyRotateBotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger("MyRotateBotHardware"),
    "state joint ", info_.joints[i].name.c_str());
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        // This arg can be either HW_IF_POSITION, HW_IF_VELOCITY, or HW_IF_EFFORT
        hardware_interface::HW_IF_POSITION,
        &hw_states_[i]
      )
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyRotateBotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger("MyRotateBotHardware"),
    "command joint ", info_.joints[i].name.c_str());
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}


return_type MyRotateBotHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "Starting ...please wait...");

  for (int i = 0; i <= hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("MyRotateBotHardware"),
      "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (uint i = 0; i < hw_states_.size(); i++) {
    if (std::isnan(hw_states_[i])) {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "System Sucessfully started!");

  return return_type::OK;
}

return_type MyRotateBotHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i <= hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("MyRotateBotHardware"),
      "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "System sucessfully stopped!");

  return return_type::OK;
}

hardware_interface::return_type MyRotateBotHardware::read()
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++) {
    // Simulate RRBot's movement
    hw_states_[i] = hw_commands_[i] + (hw_states_[i] - hw_commands_[i]) / hw_slowdown_;
    RCLCPP_INFO(
      rclcpp::get_logger("MyRotateBotHardware"),
      "Got state %.5f for joint %d!", hw_states_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "Joints sucessfully read!");

  return return_type::OK;
}

hardware_interface::return_type MyRotateBotHardware::write()
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("MyRotateBotHardware"),
      "Got command %.5f for joint %d!", hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("MyRotateBotHardware"),
    "Joints sucessfully written!");

  return return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_rotate_bot_namespace::MyRotateBotHardware,
  hardware_interface::SystemInterface
)
