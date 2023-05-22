#include "ar3_controllers/status_controller.hpp"

#include <string>

namespace ar3_controllers
{
controller_interface::CallbackReturn StatusController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<status_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration StatusController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::InterfaceConfiguration ar3_controllers::StatusController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("status/encoder_count_error");
  config.names.emplace_back("status/limit_switch_triggered");

  // TODO: Use approach in joint_state_broadcaster to get joint params
  // automatically if this needs to be more generic.
  for (unsigned int i = 1; i < 7; ++i)
  {
    std::string name = "joint" + std::to_string(i) + "/" + "limit_switch";
    config.names.push_back(name);
  }
  return config;
}

controller_interface::return_type ar3_controllers::StatusController::update(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& /*period*/)
{
  publish_status();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ar3_controllers::StatusController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void StatusController::publish_status()
{
  status_msg_.encoder_count_error = static_cast<bool>(state_interfaces_[0].get_value());
  status_msg_.limit_switch_triggered = static_cast<bool>(state_interfaces_[1].get_value());

  for (unsigned int i = 0; i < 6; ++i)
  {
    status_msg_.limit_switches_triggered[i] = static_cast<bool>(state_interfaces_[i+2].get_value());
  }
  status_pub_->publish(status_msg_);
}

controller_interface::CallbackReturn
ar3_controllers::StatusController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    // register publisher
    status_pub_ = get_node()->create_publisher<ar3_msgs::msg::Status>("~/ar3_status", rclcpp::SystemDefaultsQoS());

  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  status_msg_.limit_switches_triggered.resize(6, false);

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ar3_controllers::StatusController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    // reset publisher
    status_pub_.reset();
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace ar3_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ar3_controllers::StatusController, controller_interface::ControllerInterface)
