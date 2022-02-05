#include "ar3_hardware_driver/ar3_encoder_switch_motor_serial_comm.hpp"

#include <thread>
#include <chrono>
#include <iostream>

#define FW_VERSION "0.0.1"


namespace ar3_hardware_driver {

bool AR3EncoderSwitchMotorSerialComm::init(const std::string& device, int baudrate, const std::string& fw_version)
{
  try {
    serial_ = std::make_shared<TimeoutSerial>(
        device, baudrate,
        boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none),
        boost::asio::serial_port_base::character_size(8),
        boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none),
        boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));

    serial_->setTimeout(boost::posix_time::seconds(1)); // TODO: timeout config
    serial_->writeString("V\r\n");
    std::string result = serial_->readStringUntil("\r\n");

    // TODO: try/catch
    if (fw_version != result) {
      return false;
    }

  } catch(boost::system::system_error& e)
  {
    std::cout <<"Error: " << e.what() << std::endl;
    return false;
  }
  return true;
}

void AR3EncoderSwitchMotorSerialComm::calibrate_joints()
{
}

bool AR3EncoderSwitchMotorSerialComm::get_joint_encoder_counts(std::vector<int>& encoder_counts)
{
  // Request the joint encoder counts
  serial_->writeString("E\r\n");
  std::string jp_result;
  try {
    jp_result = serial_->readStringUntil("\r\n");
  } catch (const timeout_exception& e) {
    std::cout << "Timeout!" << std::endl;
  } catch (const boost::system::system_error& e) {
    std::cout << "Exception: " << e.what() << std::endl;
  }
  return parse_list(jp_result, ",", encoder_counts.size()+1, encoder_counts);
}

bool AR3EncoderSwitchMotorSerialComm::get_joint_positions(std::vector<double>& joint_positions)
{
  // Request the joint encoder counts
  serial_->writeString("P\r\n");
  std::string jp_result;
  try {
    jp_result = serial_->readStringUntil("\r\n");
  } catch (const timeout_exception& e) {
    std::cout << "Timeout!" << std::endl;
  } catch (const boost::system::system_error& e) {
    std::cout << "Exception: " << e.what() << std::endl;
  }
  return parse_list(jp_result, ",", joint_positions.size()+1, joint_positions);
}

bool AR3EncoderSwitchMotorSerialComm::set_joint_positions(const std::vector<double>& desired_joint_positions)
{
  std::string cmd = "D,";
  for (const auto& p : desired_joint_positions) {
    cmd += std::to_string(p) + ",";
  }
  serial_->writeString(cmd + "\r\n");
  std::string response;
  try {
    response = serial_->readStringUntil("\r\n");
  } catch (const timeout_exception& e) {
    std::cout << "Timeout!" << std::endl;
  } catch (const boost::system::system_error& e) {
    std::cout << "Exception: " << e.what() << std::endl;
  }
  return response == "d,OK";
}

} // namespace ar3_hardware_driver
