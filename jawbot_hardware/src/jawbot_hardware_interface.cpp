#include "jawbot_hardware/jawbot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace jawbot_hardware
{

hardware_interface::CallbackReturn JawbotHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse custom parameters injected from the URDF <ros2_control> block
  port_ = info_.hardware_parameters["port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  double ticks_per_rev = std::stod(info_.hardware_parameters["ticks_per_rev"]);
  ticks_per_rad_ = ticks_per_rev / (2.0 * M_PI);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JawbotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JawbotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn JawbotHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("JawbotHardware"), "Configuring serial port: %s at %d baud...", port_.c_str(), baud_rate_);
  try {
    serial_conn_.connect(port_, baud_rate_, 100); // 100ms timeout
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("JawbotHardware"), "Configuration failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JawbotHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("JawbotHardware"), "Activating hardware... Sending zeros.");
  
  // Zero out internal arrays safely
  for (uint i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  
  serial_conn_.write_msg("m 0.0 0.0\n");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JawbotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("JawbotHardware"), "Deactivating... Stopping motors.");
  serial_conn_.write_msg("m 0.0 0.0\n");
  serial_conn_.disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JawbotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!serial_conn_.is_connected()) return hardware_interface::return_type::ERROR;

  std::string response = serial_conn_.read_msg();
  if (response.empty()) return hardware_interface::return_type::OK; // Wait for next cycle

  // Parse Phase 2 Contract: "e <left> <right> t <micros>"
  int32_t current_left_ticks = 0, current_right_ticks = 0;
  uint32_t current_pico_micros = 0;
  
  if (sscanf(response.c_str(), "e %d %d t %u", &current_left_ticks, &current_right_ticks, &current_pico_micros) == 3) 
  {
    // Absolute position in radians
    hw_positions_[0] = current_left_ticks / ticks_per_rad_;
    hw_positions_[1] = current_right_ticks / ticks_per_rad_;

    // Calculate exact velocity using the Pico's precise internal hardware clock difference
    double delta_time_sec = (current_pico_micros - last_pico_micros_) / 1000000.0;
    
    if (delta_time_sec > 0) {
      hw_velocities_[0] = ((current_left_ticks - last_left_ticks_) / ticks_per_rad_) / delta_time_sec;
      hw_velocities_[1] = ((current_right_ticks - last_right_ticks_) / ticks_per_rad_) / delta_time_sec;
    }

    last_left_ticks_ = current_left_ticks;
    last_right_ticks_ = current_right_ticks;
    last_pico_micros_ = current_pico_micros;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JawbotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!serial_conn_.is_connected()) return hardware_interface::return_type::ERROR;

  // Pack the velocity commands requested by ROS 2 into the Phase 2 'm' String Contract
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "m %.3f %.3f\n", hw_commands_[0], hw_commands_[1]);
  
  serial_conn_.write_msg(std::string(buffer));

  return hardware_interface::return_type::OK;
}

}  // namespace jawbot_hardware

// Essential macro that exposes this C++ class as a dynamic loadable plugin!
PLUGINLIB_EXPORT_CLASS(
  jawbot_hardware::JawbotHardwareInterface, hardware_interface::SystemInterface)