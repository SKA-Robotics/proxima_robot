#ifndef PROXIMA_MANIP_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_
#define PROXIMA_MANIP_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace proxima_manip_controller
{

struct DynamixelJoint
{
  int servo_id = -1;
  double command_position = 0.0;
  double state_position = 0.0;
  std::string joint_name;
  std::string model;
};

class DynamixelSystem : public hardware_interface::SystemInterface
{
public:
  //ogarnij to tototoot
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int baud_rate_;
  std::string port_name_;
  std::vector<DynamixelJoint> joints_;
  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler1_;
  dynamixel::PacketHandler * packetHandler2_;

};

}  // namespace proxima_manip_controller

#endif  // PROXIMA_MANIP_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_