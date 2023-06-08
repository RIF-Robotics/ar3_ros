#ifndef AR3_HARDWARE_DRIVER__AR3_ENCODER_SWITCH_MOTOR_SERIAL_COMM_HPP_
#define AR3_HARDWARE_DRIVER__AR3_ENCODER_SWITCH_MOTOR_SERIAL_COMM_HPP_

#include <math.h>

#include <boost/tokenizer.hpp>
#include <memory>
#include <string>
#include <vector>

#include "ar3_hardware_driver/timeout_serial.h"
#include "ar3_hardware_driver/visibility_control.h"

namespace ar3_hardware_driver
{
class AR3EncoderSwitchMotorSerialComm
{
public:
  AR3_HARDWARE_DRIVER_PUBLIC
  bool init(const std::string & device, int baudrate, const std::string & fw_version);

  AR3_HARDWARE_DRIVER_PUBLIC
  bool get_joint_encoder_counts(std::vector<int> & joint_encoder_counts);

  AR3_HARDWARE_DRIVER_PUBLIC
  bool get_joint_positions(std::vector<double> & joint_positions);

  AR3_HARDWARE_DRIVER_PUBLIC
  bool get_status_bits(std::vector<unsigned int> & status_bits);

  AR3_HARDWARE_DRIVER_PUBLIC
  bool get_limit_switch_rising_edges(std::vector<double> & limit_switches);

  AR3_HARDWARE_DRIVER_PUBLIC
  bool set_joint_positions(const std::vector<double> & desired_joint_positions);

  AR3_HARDWARE_DRIVER_PUBLIC
  void calibrate_joints();

private:
  std::shared_ptr<TimeoutSerial> serial_;

  template <class T>
  bool parse_list(
    const std::string & str, const std::string & delim, const unsigned int expected_length,
    std::vector<T> & result)
  {
    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
    boost::char_separator<char> sep{delim.c_str()};
    tokenizer tok{str, sep};

    std::vector<std::string> tokens;
    for (const auto & t : tok) {
      tokens.push_back(t);
    }

    if (tokens.size() != expected_length) {
      return false;
    }

    for (unsigned int i = 1; i < tokens.size(); ++i) {
      result[i - 1] = static_cast<T>(std::stod(tokens[i]));
    }
    return true;
  }

  bool send_command(const std::string & cmd, std::string & response);
};

}  // namespace ar3_hardware_driver

#endif  // AR3_HARDWARE_DRIVER__AR3_ENCODER_SWITCH_MOTOR_SERIAL_COMM_HPP_
