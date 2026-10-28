#ifndef PROXIMA_MANIP_CONTROLLER__MOTEUS_SYSTEM_HPP_
#define PROXIMA_MANIP_CONTROLLER__MOTEUS_SYSTEM_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "moteus.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace proxima_manip_controller
{

struct MoteusJoint
{
  int can_id = -1;
  std::shared_ptr<mjbots::moteus::Controller> controller;
  mjbots::moteus::Query::Result last_result;
  bool got_answer = false;
  int cycles_without_answer = 0;

  std::string joint_position_interface;
  std::string joint_velocity_interface;
  std::string joint_effort_interface;
  double effort_max = std::numeric_limits<double>::quiet_NaN();
};

class MoteusSystem : public hardware_interface::SystemInterface
{
public:
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
  std::vector<MoteusJoint> joints_;
  std::vector<mjbots::moteus::CanFdFrame> frames_to_send_;
  std::vector<mjbots::moteus::CanFdFrame> answer_frames_;

  std::shared_ptr<mjbots::moteus::Transport> transport_;
};

}  // namespace proxima_manip_controller

#endif  // PROXIMA_MANIP_CONTROLLER__MOTEUS_SYSTEM_HPP_
