#include "factr_controller/dynamixel_sdk_interface.hpp"

#include <stdexcept>
#include <algorithm>  // std::clamp

DynamixelSdkInterface::DynamixelSdkInterface(
    const std::string &device_name,
    int baudrate,
    double protocol_version)
: device_name_(device_name),
  baudrate_(baudrate),
  protocol_version_(protocol_version),
  port_handler_(nullptr),
  packet_handler_(nullptr),
  logger_(rclcpp::get_logger("dynamixel_sdk_interface")),
  mode_a_(5),  // Current-based Position Control Mode
  mode_b_(5)   // Current-based Position Control Mode
{
  // ===== IDs =====
  dxl_ids_all_ = {1, 2, 3, 4, 5, 6, 7};
  type_a_ids_  = {2, 4};   // XM430-W210-T
  for (auto id : dxl_ids_all_) {
    if (std::find(type_a_ids_.begin(), type_a_ids_.end(), id) == type_a_ids_.end()) {
      type_b_ids_.push_back(id);  // XC330-T288-T
    }
  }
  // ===== joint bounds (deg → unit) =====
  const double DEG2UNIT_LOCAL = DEG2UNIT;
  std::vector<double> lower_deg = {-45, -90, -180,   0, -270, -180, -90};
  std::vector<double> upper_deg = { 45, 105,  270, 120,  270,  -15,  90};
  lower_bound_unit_.resize(lower_deg.size());
  upper_bound_unit_.resize(upper_deg.size());
  for (std::size_t i = 0; i < lower_deg.size(); ++i) {
    lower_bound_unit_[i] = static_cast<int32_t>(lower_deg[i] * DEG2UNIT_LOCAL);
    upper_bound_unit_[i] = static_cast<int32_t>(upper_deg[i] * DEG2UNIT_LOCAL);
  }
  
  // ===== control table (MATLAB과 동일) =====
  addr_torque_enable_        = 64;
  addr_operating_mode_       = 11;
  addr_profile_acc_          = 108;
  addr_profile_vel_          = 112;
  addr_goal_position_        = 116;
  addr_present_current_      = 126;
  addr_present_velocity_     = 128;
  addr_present_position_     = 132;

  len_torque_enable_         = 1;
  len_operating_mode_        = 1;
  len_profile_acc_           = 4;
  len_profile_vel_           = 4;
  len_goal_position_         = 4;
  len_present_current_       = 2;
  len_present_velocity_      = 4;
  len_present_position_      = 4;
  len_present_all_           = 10;

  torque_enable_value_  = 1;
  torque_disable_value_ = 0;
  // ===== SDK 초기화 =====
  initPort();
  initMotors();
  initBulkRead();
  // BulkWrite 준비 (goal position 쓰기용)
  group_bulk_write_ =
      std::make_unique<dynamixel::GroupBulkWrite>(port_handler_, packet_handler_);
  RCLCPP_INFO(logger_, "DynamixelSdkInterface initialized");
}

DynamixelSdkInterface::~DynamixelSdkInterface()
{
  try {
    setTorqueEnabled(false);
  } catch (...) {
    RCLCPP_WARN(logger_, "Exception while disabling torque in destructor");
  }

  if (port_handler_ != nullptr) {
    port_handler_->closePort();
  }
}

void DynamixelSdkInterface::initPort()
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  if (!port_handler_->openPort()) {
    throw std::runtime_error("Failed to open port: " + device_name_);
  }
  if (!port_handler_->setBaudRate(baudrate_)) {
    throw std::runtime_error("Failed to set baudrate");
  }

  RCLCPP_INFO(logger_, "Opened port %s @ %d", device_name_.c_str(), baudrate_);
}

void DynamixelSdkInterface::initMotors()
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // torque off (MATLAB: write1ByteTxRx(..., TORQUE_DISABLE))
  for (auto id : dxl_ids_all_) {
    dxl_comm_result = packet_handler_->write1ByteTxRx(
        port_handler_, id, addr_torque_enable_, torque_disable_value_, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(logger_, "Torque OFF failed (ID=%d): %s",
                  id, packet_handler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_WARN(logger_, "Torque OFF error (ID=%d): %s",
                  id, packet_handler_->getRxPacketError(dxl_error));
    }
  }
  // TYPE A: operating mode
  for (auto id : type_a_ids_) {
    packet_handler_->write1ByteTxRx(
        port_handler_, id, addr_operating_mode_, mode_a_, &dxl_error);
  }

  // TYPE B: operating mode 
  for (auto id : type_b_ids_) {
    packet_handler_->write1ByteTxRx(
        port_handler_, id, addr_operating_mode_, mode_b_, &dxl_error);
  }

  // torque on
  setTorqueEnabled(true);
}

void DynamixelSdkInterface::initBulkRead()
{
  group_bulk_read_ =
      std::make_unique<dynamixel::GroupBulkRead>(port_handler_, packet_handler_);

  for (auto id : dxl_ids_all_) {
    bool ok = true;
    ok &= group_bulk_read_->addParam(id, addr_present_current_,  len_present_all_);
    if (!ok) {
      RCLCPP_WARN(logger_, "GroupBulkRead addParam failed (ID=%d)", id);
    }
  }

  RCLCPP_INFO(logger_, "GroupBulkRead setup done for %zu joints",
              dxl_ids_all_.size());
}

void DynamixelSdkInterface::setTorqueEnabled(bool enable)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t value = enable ? torque_enable_value_ : torque_disable_value_;

  for (auto id : dxl_ids_all_) {
    dxl_comm_result = packet_handler_->write1ByteTxRx(
        port_handler_, id, addr_torque_enable_, value, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(logger_, "Torque %s failed (ID=%d): %s",
                  enable ? "ON" : "OFF",
                  id, packet_handler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_WARN(logger_, "Torque %s error (ID=%d): %s",
                  enable ? "ON" : "OFF",
                  id, packet_handler_->getRxPacketError(dxl_error));
    }
  }
}

bool DynamixelSdkInterface::readOnce(State &out_state)
{
  if (!group_bulk_read_) {
    RCLCPP_ERROR(logger_, "GroupBulkRead not initialized");
    return false;
  }

  int dxl_comm_result = group_bulk_read_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(logger_, "BulkRead txRxPacket failed: %s",
                packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  const std::size_t n = dxl_ids_all_.size();
  out_state.position.assign(n, 0);
  out_state.velocity.assign(n, 0);
  out_state.current.assign(n, 0);

  for (std::size_t i = 0; i < n; ++i) {
    auto id = dxl_ids_all_[i];

    // position (uint32 → int32)
    if (group_bulk_read_->isAvailable(
            id, addr_present_position_, len_present_position_)) {
      uint32_t raw = group_bulk_read_->getData(
          id, addr_present_position_, len_present_position_);
      out_state.position[i] = static_cast<int32_t>(raw);
    } else {
      RCLCPP_WARN(logger_, "Position not available (ID=%d)", id);
    }

    // velocity (uint32 → int32)
    if (group_bulk_read_->isAvailable(
            id, addr_present_velocity_, len_present_velocity_)) {
      uint32_t raw = group_bulk_read_->getData(
          id, addr_present_velocity_, len_present_velocity_);
      out_state.velocity[i] = static_cast<int32_t>(raw);
    } else {
      RCLCPP_WARN(logger_, "Velocity not available (ID=%d)", id);
    }

    // current (uint32 → uint16 → int16)
    if (group_bulk_read_->isAvailable(
            id, addr_present_current_, len_present_current_)) {
      uint32_t raw  = group_bulk_read_->getData(
          id, addr_present_current_, len_present_current_);
      uint16_t raw16 = static_cast<uint16_t>(raw);
      out_state.current[i] = static_cast<int16_t>(raw16);
    } else {
      RCLCPP_WARN(logger_, "Current not available (ID=%d)", id);
    }
  }

  return true;
}

bool DynamixelSdkInterface::writeGoalPositions(
    const std::vector<int32_t> &goals_unit)
{
  if (!group_bulk_write_) {
    RCLCPP_ERROR(logger_, "GroupBulkWrite not initialized");
    return false;
  }
  if (goals_unit.size() != dxl_ids_all_.size()) {
    RCLCPP_ERROR(logger_, "Goal size mismatch (got %zu, expected %zu)",
                 goals_unit.size(), dxl_ids_all_.size());
    return false;
  }

  group_bulk_write_->clearParam();

  for (std::size_t i = 0; i < dxl_ids_all_.size(); ++i) {
    auto id = dxl_ids_all_[i];
    int32_t g = goals_unit[i];

    // 안전을 위해 bound 클리핑 (MATLAB에서 clip 사용했던 부분)
    g = std::clamp(g, lower_bound_unit_[i], upper_bound_unit_[i]);

    uint8_t param_goal[4];
    param_goal[0] = static_cast<uint8_t>(g & 0xFF);
    param_goal[1] = static_cast<uint8_t>((g >> 8) & 0xFF);
    param_goal[2] = static_cast<uint8_t>((g >> 16) & 0xFF);
    param_goal[3] = static_cast<uint8_t>((g >> 24) & 0xFF);

    bool ok = group_bulk_write_->addParam(
        id, addr_goal_position_, len_goal_position_, param_goal);
    if (!ok) {
      RCLCPP_WARN(logger_, "GroupBulkWrite addParam failed (ID=%d)", id);
    }
  }

  int dxl_comm_result = group_bulk_write_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(logger_, "BulkWrite txPacket failed: %s",
                packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  group_bulk_write_->clearParam();
  return true;
}

bool DynamixelSdkInterface::writeGoalPositionsDeg(
    const std::vector<double> &goals_deg)
{
  if (goals_deg.size() != dxl_ids_all_.size()) {
    RCLCPP_ERROR(logger_, "Goal(deg) size mismatch (got %zu, expected %zu)",
                 goals_deg.size(), dxl_ids_all_.size());
    return false;
  }

  std::vector<int32_t> goals_unit(goals_deg.size());
  for (std::size_t i = 0; i < goals_deg.size(); ++i) {
    goals_unit[i] = static_cast<int32_t>(goals_deg[i] * DEG2UNIT);
  }

  return writeGoalPositions(goals_unit);
}
