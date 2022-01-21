#include "ar3_hardware_driver/ar3_encoder_switch_motor_serial_comm.hpp"

#include <thread>
#include <chrono>
#include <iostream>

#include <boost/tokenizer.hpp>

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
    serial_->writeString("FV\r\n");
    // TODO: try/catch
    if (fw_version != serial_->readStringUntil("\r\n")) {
      return false;
    }

  } catch(boost::system::system_error& e)
  {
    std::cout <<"Error: " << e.what() << std::endl;
    return false;
  }

  encoder_steps_per_rad_ = {
    {
      to_radians(227.5555555555556),
      to_radians(284.4444444444444),
      to_radians(284.4444444444444),
      to_radians(223.0044444444444),
      to_radians(56.04224675948152),
      to_radians(108.0888888888889)
    }
  };

  return true;
}

void AR3EncoderSwitchMotorSerialComm::set_stepper_speed(std::vector<double>& max_speed, std::vector<double>& max_accel)
{
}

// Update between hardware interface and hardware driver
void AR3EncoderSwitchMotorSerialComm::update(std::vector<double>& pos_commands, std::vector<double>& joint_positions)
{
}

void AR3EncoderSwitchMotorSerialComm::calibrate_joints()
{
}

void AR3EncoderSwitchMotorSerialComm::get_joint_encoder_counts(std::vector<int>& encoder_counts)
{
  // Request the joint encoder counts
  serial_->writeString("EC\r\n");
  std::string jp_result;
  try {
    jp_result = serial_->readStringUntil("\r\n");
  } catch (timeout_exception e) {
    std::cout << "Timeout!" << std::endl;
  } catch (boost::system::system_error e) {
    std::cout << "Exception: " << e.what() << std::endl;
  }

  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};
  tokenizer tok {jp_result, sep};

  std::vector<std::string> tokens;
  tokens.reserve(encoder_counts.size());
  for (const auto &t : tok) {
    tokens.push_back(t);
  }

  if (tokens.size() == encoder_counts.size()) {
    for (unsigned int i = 0; i < encoder_counts.size(); ++i) {
      encoder_counts[i] = std::stoi(tokens[i]);
    }
  } else {
    std::cout << "Error parsing joint positions" << std::endl;
  }
}


// Send msg to board and collect data
void AR3EncoderSwitchMotorSerialComm::exchange(std::string outMsg)
{
}

bool AR3EncoderSwitchMotorSerialComm::transmit(const std::string& msg, std::string& err)
{
  return true;
}

bool AR3EncoderSwitchMotorSerialComm::receive(std::string& inMsg)
{
  return true;
}

void AR3EncoderSwitchMotorSerialComm::update_encoder_calibrations(const std::string& msg)
{
  //size_t idx1 = msg.find("A", 2) + 1;
  //size_t idx2 = msg.find("B", 2) + 1;
  //size_t idx3 = msg.find("C", 2) + 1;
  //size_t idx4 = msg.find("D", 2) + 1;
  //size_t idx5 = msg.find("E", 2) + 1;
  //size_t idx6 = msg.find("F", 2) + 1;
  //enc_calibrations_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
  //enc_calibrations_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
  //enc_calibrations_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
  //enc_calibrations_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
  //enc_calibrations_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
  //enc_calibrations_[5] = std::stoi(msg.substr(idx6));

  // @TODO update config file
  //ROS_INFO("Successfully updated encoder calibrations");
}

void AR3EncoderSwitchMotorSerialComm::update_encoder_steps(const std::string& msg)
{
  //size_t idx1 = msg.find("A", 2) + 1;
  //size_t idx2 = msg.find("B", 2) + 1;
  //size_t idx3 = msg.find("C", 2) + 1;
  //size_t idx4 = msg.find("D", 2) + 1;
  //size_t idx5 = msg.find("E", 2) + 1;
  //size_t idx6 = msg.find("F", 2) + 1;
  //enc_steps_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
  //enc_steps_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
  //enc_steps_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
  //enc_steps_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
  //enc_steps_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
  //enc_steps_[5] = std::stoi(msg.substr(idx6));
}

void AR3EncoderSwitchMotorSerialComm::enc_steps_to_joint_pos(const std::vector<int>& enc_steps, std::vector<double>& joint_positions)
{
  //for (unsigned int i = 0; i < enc_steps.size(); ++i)
  //{
  //  // convert enc steps to joint deg
  //  joint_positions[i] = static_cast<double>(enc_steps[i]) / enc_steps_per_deg_[i];
  //}
}

void AR3EncoderSwitchMotorSerialComm::joint_pos_to_enc_steps(const std::vector<double>& joint_positions, std::vector<int>& enc_steps)
{
  //for (unsigned int i = 0; i < joint_positions.size(); ++i)
  //{
  //  // convert joint deg to enc steps
  //  enc_steps[i] = static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i]);
  //}
}

} // namespace ar3_hardware_driver
