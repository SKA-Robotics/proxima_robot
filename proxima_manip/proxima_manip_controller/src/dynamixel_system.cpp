#include "proxima_manip_controller/dynamixel_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
//socketCAN
#include <linux/can.h>
#include <sys/socket.h>
#include <net/if.h>
#include <cmath> 

//adresy akcji z firmware'u
#define MSG_MOTOR_COMMAND     0x01
#define MSG_ACTION_REQUEST    0x02
#define MSG_STATUS_REPORT     0x03
#define MSG_AXIS_REPORT       0x04
#define MSG_MOTOR_REPORT      0x05

#define CTRLSIGNAL_POSITION 0x02

#define ACTION_ARM 0x00
#define ACTION_DISARM 0x01
#define ACTION_SOFTWARE_RESET 0x05


namespace proxima_manip_controller
{

hardware_interface::CallbackReturn DynamixelSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters.at("port_name");
  joints_.clear();

  for (const auto & joint : info_.joints){
    DynamixelJoint new_joint;
    new_joint.joint_name = joint.name;

    try
    {
      new_joint.servo_id = std::stoi(joint.parameters.at("servo_id"));
      new_joint.model = joint.parameters.at("model");
    }
    catch (const std::exception& e){
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "no parameter 'servo_id' for joint %s, or model", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    new_joint.command_position = 0.0;
    new_joint.state_position = 0.0;

    joints_.push_back(new_joint);
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (auto & joint : joints_) {

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.joint_name,
        hardware_interface::HW_IF_VELOCITY,
        &joint.state_velocity)
    );

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.joint_name, 
        hardware_interface::HW_IF_POSITION, 
        &joint.state_position)
    );
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (auto & joint : joints_) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.joint_name, 
        hardware_interface::HW_IF_POSITION, 
        &joint.command_position)
    );
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DynamixelSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //otwarcie gniazda dla cana
  can_socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "Cannot create CAN socket");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // obiernamy numer socketa cana nadanego przez linuxa
  unsigned int ifindex = if_nametoindex(port_name_.c_str());
  if (ifindex == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "No interface found %s", port_name_.c_str());
    ::close(can_socket_);
    can_socket_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // robisz tego structa dla linuxa zeby wiedzial o adresie kabla do binda potrzebne
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifindex;

  if (::bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "error bind() na %s", port_name_.c_str());
    ::close(can_socket_);
    can_socket_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "CAN (%s) open.", port_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "activation (ACTION_ARM)");

  struct can_frame frame{};
  frame.can_id = (0 << 5) | MSG_ACTION_REQUEST;
  frame.can_dlc = 1;
  frame.data[0] = ACTION_ARM;

  if (::write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "error for command ACTION_ARM");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "deactivation (ACTION_DISARM)");

  struct can_frame frame{};
  frame.can_id = (0 << 5) | MSG_ACTION_REQUEST;
  frame.can_dlc = 1;
  frame.data[0] = ACTION_DISARM;

  ::write(can_socket_, &frame, sizeof(frame));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (can_socket_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  struct can_frame frame{};

  while (::read(can_socket_, &frame, sizeof(frame)) == sizeof(frame)) {
    int node_id = frame.can_id >> 5;                 //usuniecie ostatmich 5 bitow
    int msg_id  = frame.can_id & 0x1F;              //czytanie wszystkich ramek

    if (msg_id != MSG_AXIS_REPORT || frame.can_dlc < 8) {
      continue;
    }

    uint8_t pos_bytes[4] = {frame.data[3], frame.data[2], frame.data[1], frame.data[0]};
    uint8_t vel_bytes[4] = {frame.data[7], frame.data[6], frame.data[5], frame.data[4]};

    float pos_rot = 0.0f;
    float vel_rot = 0.0f;
    std::memcpy(&pos_rot, pos_bytes, sizeof(float));
    std::memcpy(&vel_rot, vel_bytes, sizeof(float));

    for (auto & joint : joints_) {        //na obroty przeliczanie
      if (joint.servo_id == node_id) {
        joint.state_position = static_cast<double>(pos_rot) * 2.0 * M_PI;
        joint.state_velocity = static_cast<double>(vel_rot) * 2.0 * M_PI;
        break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (can_socket_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  for (auto & joint : joints_) {
    struct can_frame frame{};

    float pos_rot = static_cast<float>(joint.command_position / (2.0 * M_PI));

    uint8_t bytes[4];
    std::memcpy(bytes, &pos_rot, sizeof(float));

    if (joint.servo_id == 0x2c) {
      frame.can_id = (0x2c << 5);
      frame.can_dlc = 4;
      frame.data[0] = bytes[3];
      frame.data[1] = bytes[2];
      frame.data[2] = bytes[1];
      frame.data[3] = bytes[0];
    } else {
      frame.can_id = (joint.servo_id << 5) | MSG_MOTOR_COMMAND;    //przesuniecie od ostatmniego do przodu o 5 id silnika
      frame.can_dlc = 8;
      frame.data[0] = CTRLSIGNAL_POSITION;  // tryb pozycji (CTRLSIGNAL_POSITION)
      frame.data[1] = bytes[3];
      frame.data[2] = bytes[2];
      frame.data[3] = bytes[1];
      frame.data[4] = bytes[0];
    }

    if (::write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
      RCLCPP_WARN(
        rclcpp::get_logger("DynamixelSystem"),
        "Błąd wysyłania ramki CAN dla jointa %s (ID: %d)",
        joint.joint_name.c_str(), joint.servo_id);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace proxima_manip_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  proxima_manip_controller::DynamixelSystem, hardware_interface::SystemInterface)