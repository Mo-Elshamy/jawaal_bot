#ifndef JAWBOT_HARDWARE__JAWBOT_HARDWARE_INTERFACE_HPP_
#define JAWBOT_HARDWARE__JAWBOT_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// Include your custom serial driver
#include "jawbot_hardware/serial_comms.hpp"

namespace jawbot_hardware
{

class JawbotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JawbotHardwareInterface)

  // Initialization and parsing URDF parameters
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Real-time export of pointers to the controller_manager
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Lifecycle state transitions
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // The 50Hz Real-Time Read/Write loops
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  SerialComms serial_conn_;
  
  // Internal ROS states
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Tracking variables for exact velocity calculation from Pico's microsecond clock
  int32_t last_left_ticks_ = 0;
  int32_t last_right_ticks_ = 0;
  uint32_t last_pico_micros_ = 0;

  double ticks_per_rad_ = 0.0;
  std::string port_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;
};

}  // namespace jawbot_hardware

#endif  // JAWBOT_HARDWARE__JAWBOT_HARDWARE_INTERFACE_HPP_