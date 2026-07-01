#ifndef JAWBOT_HARDWARE__SERIAL_COMMS_HPP_
#define JAWBOT_HARDWARE__SERIAL_COMMS_HPP_

#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

namespace jawbot_hardware
{

class SerialComms
{
public:
  SerialComms() = default;
  ~SerialComms();

  // Connects to the Pico at /dev/ttyACM0 (or similar) at 115200 baud
  void connect(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms);
  
  void disconnect();
  bool is_connected() const;

  // Reads the incoming "e <left> <right> t <micros>\n" string from the Pico
  std::string read_msg();

  // Sends the "m <left_vel> <right_vel>\n" string down to the Pico
  void write_msg(const std::string & msg_to_send);

private:
  int serial_fd_ = -1; // Linux file descriptor for the port
};

}  // namespace jawbot_hardware

#endif  // JAWBOT_HARDWARE__SERIAL_COMMS_HPP_