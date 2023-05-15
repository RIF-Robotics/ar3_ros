#ifndef AR3_CONTROLLERS__STATUS_CONTROLLER_HPP_
#define AR3_CONTROLLERS__STATUS_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include <ar3_msgs/msg/status.hpp>

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

namespace ar3_controllers
{

class StatusController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  void publish_status();

protected:
  std::shared_ptr<rclcpp::Publisher<ar3_msgs::msg::Status>> status_pub_;
  ar3_msgs::msg::Status status_msg_;

};
}  // namespace ar3_controllers

#endif  // AR3_CONTROLLERS__STATUS_CONTROLLER_HPP_
