#include "ar3_hardware_driver/ar3_encoder_switch_motor_serial_comm.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#define FW_VERSION "0.0.1"

namespace ar3_hardware_driver
{

bool AR3EncoderSwitchMotorSerialComm::init(
  const std::string & device, int baudrate, const std::string & fw_version)
{
  try {
    serial_ = std::make_shared<TimeoutSerial>(
      device, baudrate,
      boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
      boost::asio::serial_port_base::character_size(8),
      boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none),
      boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    serial_->setTimeout(boost::posix_time::seconds(1));  // TODO: timeout config

    std::string response;
    if (!send_command("V", response)) {
      return false;
    }

    if (fw_version != response) {
      return false;
    }

  } catch (boost::system::system_error & e) {
    std::cout << "Error: " << e.what() << std::endl;
    return false;
  }
  return true;
}

void AR3EncoderSwitchMotorSerialComm::calibrate_joints() {}

bool AR3EncoderSwitchMotorSerialComm::get_joint_encoder_counts(std::vector<int> & encoder_counts)
{
  std::string response;
  if (!send_command("E", response)) {
    return false;
  }
  return parse_list(response, ",", encoder_counts.size() + 1, encoder_counts);
}

bool AR3EncoderSwitchMotorSerialComm::get_joint_positions(std::vector<double> & joint_positions)
{
  std::string response;
  if (!send_command("P", response)) {
    return false;
  }
  return parse_list(response, ",", joint_positions.size() + 1, joint_positions);
}

bool AR3EncoderSwitchMotorSerialComm::get_status_bits(std::vector<unsigned int> & status_bits)
{
  std::string response;
  if (!send_command("Q", response)) {
    return false;
  }
  return parse_list(response, ",", 3, status_bits);
}

bool AR3EncoderSwitchMotorSerialComm::get_limit_switch_rising_edges(
  std::vector<double> & limit_switches)
{
  std::string response;
  if (!send_command("W", response)) {
    return false;
  }
  return parse_list(response, ",", limit_switches.size() + 1, limit_switches);
}

bool AR3EncoderSwitchMotorSerialComm::set_joint_positions(
  const std::vector<double> & desired_joint_positions)
{
  std::string cmd = "D,";
  for (const auto & p : desired_joint_positions) {
    cmd += std::to_string(p) + ",";
  }

  std::string response;
  if (!send_command(cmd, response)) {
    return false;
  }
  return response == "d,OK";
}

bool AR3EncoderSwitchMotorSerialComm::send_command(const std::string & cmd, std::string & response)
{
  serial_->writeString(cmd + "\n");
  try {
    response = serial_->readStringUntil("\r\n");
    return true;
  } catch (const timeout_exception & e) {
    std::cout << "Timeout!" << std::endl;
  } catch (const boost::system::system_error & e) {
    std::cout << "Exception: " << e.what() << std::endl;
  }
  return false;
}

}  // namespace ar3_hardware_driver
