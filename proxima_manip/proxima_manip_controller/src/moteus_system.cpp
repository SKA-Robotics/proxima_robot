#include "proxima_manip_controller/moteus_system.hpp"

#include <exception>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace proxima_manip_controller
{

hardware_interface::CallbackReturn MoteusSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 3) {
    RCLCPP_ERROR(get_logger(), "Invalid number of Moteus joints.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  for (const auto & joint : info_.joints) {
    MoteusJoint new_joint = MoteusJoint();

    int can_id;
    double effort_max;

    try {
      can_id = std::stoi(joint.parameters.at("can_id"));
      effort_max = std::stod(joint.parameters.at("effort_max"));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Invalid type of moteus config variables.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interface. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
          joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    new_joint.joint_position_interface = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    new_joint.joint_velocity_interface = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    new_joint.joint_effort_interface = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    new_joint.can_id = can_id;
    new_joint.effort_max = effort_max;

    joints_.push_back(new_joint);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  try {
    transport_ = mjbots::moteus::Controller::MakeSingletonTransport({});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_) {
    mjbots::moteus::Controller::Options options;
    options.id = joint.can_id;
    options.transport = transport_;

    joint.controller = std::make_shared<mjbots::moteus::Controller>(options);
  }

  for (auto & joint : joints_) {
    mjbots::moteus::Optional<mjbots::moteus::Controller::Result> result;
    try {
      result = joint.controller->SetStop();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "CAN error for id %d: %s", joint.can_id, e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!result) {
      RCLCPP_ERROR(get_logger(), "No reply from moteus id %d.", joint.can_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & v = result->values;
    if (v.mode == mjbots::moteus::Mode::kFault) {
      RCLCPP_ERROR(get_logger(), "Moteus id %d in fault, code %d.", joint.can_id, v.fault);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      get_logger(), "Moteus id %d OK: pos=%.3f rev, Vbus=%.1f V, temp=%.1f C", joint.can_id,
      v.position, v.voltage, v.temperature);
    joint.last_result = v;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MoteusSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  constexpr int kMaxCyclesWithoutAnswer = 10;

  std::vector<mjbots::moteus::CanFdFrame> frames;
  for (auto & joint : joints_) {
    joint.got_answer = false;
    frames.push_back(joint.controller->MakeQuery());
  }

  std::vector<mjbots::moteus::CanFdFrame> replies;

  try {
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "CAN transport error: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & frame : replies) {
    for (auto & joint : joints_) {
      if (frame.source != joint.can_id) {
        continue;
      }
      joint.last_result = mjbots::moteus::Query::Parse(frame.data, frame.size);
      joint.got_answer = true;
    }
  }

  for (auto & joint : joints_) {
    if (joint.got_answer) {
      if (joint.cycles_without_answer > 0) {
        RCLCPP_INFO(get_logger(), "Moteus id %d answering again.", joint.can_id);
      }
      joint.cycles_without_answer = 0;
    } else {
      joint.cycles_without_answer++;
      if (joint.cycles_without_answer == 1) {
        RCLCPP_WARN(get_logger(), "No reply from moteus id %d.", joint.can_id);
      }
      if (joint.cycles_without_answer >= kMaxCyclesWithoutAnswer) {
        RCLCPP_ERROR(
          get_logger(), "Moteus id %d silent for %d cycles, giving up.", joint.can_id,
          joint.cycles_without_answer);
        return hardware_interface::return_type::ERROR;
      }
    }

    if (joint.last_result.mode == mjbots::moteus::Mode::kFault) {
      RCLCPP_ERROR(
        get_logger(), "Moteus id %d in fault, code %d.", joint.can_id, joint.last_result.fault);
      return hardware_interface::return_type::ERROR;
    }
  }

  constexpr double kTwoPi = 2.0 * M_PI;
  for (auto & joint : joints_) {
    set_state(joint.joint_position_interface, joint.last_result.position * kTwoPi);  // rev -> rad
    set_state(
      joint.joint_velocity_interface, joint.last_result.velocity * kTwoPi);  // rev/s -> rad/s
    set_state(joint.joint_effort_interface, joint.last_result.torque);       // Nm
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteusSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: implement
  return hardware_interface::return_type::OK;
}

}  // namespace proxima_manip_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(proxima_manip_controller::MoteusSystem, hardware_interface::SystemInterface)
