#include "proxima_manip_controller/dynamixel_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp" //do  wbudowane stale tekstowe dla interfejsu pozycji
#include "rclcpp/rclcpp.hpp" //do logow
#include "dynamixel_sdk/dynamixel_sdk.h"
// https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/example/protocol1.0/read_write/read_write.cpp
// https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/example/protocol2.0/read_write/read_write.cpp
// !! na sztywno wpisane modele silnikow

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

  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
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
  RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "konfiguracja portu: %s", port_name_.c_str());

  portHandler_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
  packetHandler1_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  packetHandler2_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!portHandler_->openPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "blad: nie mozna otworzyc portu %s", port_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!portHandler_->setBaudRate(baud_rate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "blad: nie mozna ustawic predkosci %d", baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "port otwarty.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DynamixelSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "aktywacja");

  //control table
  const uint16_t ADDR_AX_TORQUE_ENABLE = 24; // 1.0
  const uint16_t ADDR_MX_TORQUE_ENABLE = 64; // 2.0
  const uint8_t TORQUE_ON = 1;

for (auto & joint : joints_) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (joint.model == "ax") {
    dxl_comm_result = packetHandler1_->write1ByteTxRx(
      portHandler_, joint.servo_id, ADDR_AX_TORQUE_ENABLE, TORQUE_ON, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "blad aktywacji serwa id %d: %s",
                   joint.servo_id, packetHandler1_->getTxRxResult(dxl_comm_result));
      return hardware_interface::CallbackReturn::ERROR;
    }
    else if (dxl_error != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "blad sprzetowy serwa id %d: %s",
                   joint.servo_id, packetHandler1_->getRxPacketError(dxl_error));
    }
  }
  else if (joint.model == "mx" || joint.model == "xm") {
    dxl_comm_result = packetHandler2_->write1ByteTxRx(
      portHandler_, joint.servo_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ON, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "blad aktywacji serwa id %d: %s",
                   joint.servo_id, packetHandler2_->getTxRxResult(dxl_comm_result));
      return hardware_interface::CallbackReturn::ERROR;
    }
    else if (dxl_error != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), "blad sprzetowy serwa id %d: %s",
                   joint.servo_id, packetHandler2_->getRxPacketError(dxl_error));
    }
  }
}

  RCLCPP_INFO(rclcpp::get_logger("DynamixelSystem"), "wszystkie silniki dzialaja");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: implement
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
  //z readwrite.cpp robotis
  int dxl_comm_result = COMM_TX_FAIL; // udana komunikacja lub nie
  uint8_t dxl_error = 0;              //bledy
  uint16_t dxl1_present_position = 0;  // dla protocol1
  uint32_t dxl2_present_position = 0;   //dla protocol2

  //linia 44
  const uint16_t ADDR_AX_PRESENT_POSITION = 36; 
  const uint32_t ADDR_X_PRESENT_POSITION = 132; //dla mx i xm

  for (auto & joint : joints_) {
    
    if (joint.model == "ax") {
      
      // linia 191
      dxl_comm_result = packetHandler1_->read2ByteTxRx(
        portHandler_, 
        joint.servo_id, 
        ADDR_AX_PRESENT_POSITION, 
        &dxl1_present_position, 
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), 
          "Błąd komunikacji przy odczycie ID: %d. %s", 
          joint.servo_id, packetHandler1_->getTxRxResult(dxl_comm_result));
      } 
      else if (dxl_error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), 
          "Błąd sprzętowy silnika ID: %d. %s", 
          joint.servo_id, packetHandler1_->getRxPacketError(dxl_error));
      } 
      else {
        // na radiany trzeba
        joint.state_position = static_cast<double>(dxl1_present_position);
      }

    } 
    else if (joint.model == "xm" || joint.model == "mx") {

      dxl_comm_result = packetHandler2_->read4ByteTxRx(
        portHandler_, 
        joint.servo_id, 
        ADDR_X_PRESENT_POSITION, 
        &dxl2_present_position,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"),
        "Błąd komunikacji przy odczycie ID: %d. %s", 
        joint.servo_id, packetHandler2_->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0){
        RCLCPP_ERROR(rclcpp::get_logger("DynamixelSystem"), 
        "Błąd sprzetu silnika id: %d. %s",
        joint.servo_id, packetHandler2_->getRxPacketError(dxl_error));
      }
      else {
        //na radiany
        joint.state_position = static_cast<double>(dxl2_present_position);
      }
        
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace proxima_manip_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  proxima_manip_controller::DynamixelSystem, hardware_interface::SystemInterface)