#ifndef AR3_HARDWARE_DRIVER__AR3_ENCODER_SWITCH_MOTOR_SERIAL_COMM_HPP_
#define AR3_HARDWARE_DRIVER__AR3_ENCODER_SWITCH_MOTOR_SERIAL_COMM_HPP_

#include <string>
#include <vector>

#include <boost/asio.hpp>

#include "ar3_hardware_driver/visibility_control.h"

namespace ar3_hardware_driver
{
class AR3EncoderSwitchMotorSerialComm
{
 public:
  AR3EncoderSwitchMotorSerialComm();

  AR3_HARDWARE_DRIVER_PUBLIC
  bool init(const std::string& device, int baudrate, int num_joints,
            const std::vector<double>& enc_steps_per_deg);

  AR3_HARDWARE_DRIVER_PUBLIC
  void set_stepper_speed(std::vector<double>& max_speed,
                         std::vector<double>& max_accel);

  AR3_HARDWARE_DRIVER_PUBLIC
  void update(std::vector<double>& pos_commands,
              std::vector<double>& joint_states);

  AR3_HARDWARE_DRIVER_PUBLIC
  void get_joint_positions(std::vector<double>& joint_positions);

  AR3_HARDWARE_DRIVER_PUBLIC
  void calibrate_joints();

 private:
  bool initialised_;
  std::string version_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  int num_joints_;
  std::vector<int> enc_commands_;
  std::vector<int> enc_steps_;
  std::vector<double> enc_steps_per_deg_;
  std::vector<int> enc_calibrations_;


  void exchange(std::string outMsg);
  bool transmit(const std::string& outMsg, std::string& err);
  bool receive(std::string &inMsg);

  void check_init(const std::string& msg);
  void update_encoder_calibrations(const std::string& msg);
  void update_encoder_steps(const std::string& msg);

  void enc_steps_to_joint_pos(const std::vector<int>& enc_steps, std::vector<double>& joint_positions);
  void joint_pos_to_enc_steps(const std::vector<double>& joint_positions, std::vector<int>& enc_steps);
};

}  // ar3_hardware_driver

#endif  // AR3_HARDWARE_DRIVER__AR3_ENCODER_SWITCH_MOTOR_SERIAL_COMM_HPP_