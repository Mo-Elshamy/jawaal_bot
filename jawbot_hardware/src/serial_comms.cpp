#include "jawbot_hardware/serial_comms.hpp"
#include <cstring>
#include <stdexcept>

namespace jawbot_hardware
{

SerialComms::~SerialComms()
{
  disconnect();
}

void SerialComms::connect(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  // 1. Open the serial port (O_RDWR = Read/Write, O_NOCTTY = Don't make it the controlling terminal)
  serial_fd_ = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  if (serial_fd_ < 0) {
    throw std::runtime_error("Failed to open serial port: " + serial_device);
  }

  // 2. Configure POSIX Terminal Control (termios)
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    throw std::runtime_error("Error from tcgetattr: " + std::string(strerror(errno)));
  }

  // Set Baud Rate (Defaulting to 115200 to match the Pico)
  speed_t speed = B115200; 
  if (baud_rate == 115200) { speed = B115200; }
  else if (baud_rate == 57600) { speed = B57600; }
  else if (baud_rate == 9600) { speed = B9600; }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1 standard configuration (8 bits, no parity, 1 stop bit)
  tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls, enable reading
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;                 // 8-bit characters
  tty.c_cflag &= ~PARENB;             // No parity bit
  tty.c_cflag &= ~CSTOPB;             // Only need 1 stop bit
  tty.c_cflag &= ~CRTSCTS;            // No hardware flow control

  // Raw processing (no canonical formatting, echo, or signaling)
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST; // Raw output

  // Read timeouts (Non-blocking)
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = timeout_ms / 100; // VTIME is in deciseconds

  // Apply settings
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("Error from tcsetattr: " + std::string(strerror(errno)));
  }
}

void SerialComms::disconnect()
{
  if (is_connected()) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool SerialComms::is_connected() const
{
  return serial_fd_ >= 0;
}

std::string SerialComms::read_msg()
{
  if (!is_connected()) return "";

  std::string received_msg = "";
  char buffer[256];
  
  // Read until we hit a newline character '\n'
  int n = read(serial_fd_, &buffer, sizeof(buffer));
  if (n > 0) {
    received_msg = std::string(buffer, n);
  }
  return received_msg;
}

void SerialComms::write_msg(const std::string & msg_to_send)
{
  if (!is_connected()) return;
  
  // Send the command string down the USB cable
  write(serial_fd_, msg_to_send.c_str(), msg_to_send.length());
}

}  // namespace jawbot_hardware