#pragma once

#include <vector>
#include <string>
#include <memory>
#include <numbers>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Dynamixel SDK를 직접 사용해서
// - 포트/보드레이트 설정
// - Torque ON/OFF
// - Operating mode / Profile 설정
// - GroupBulkRead로 pos/vel/current 읽기
// - GroupBulkWrite로 goal position 쓰기
// 를 담당하는 유틸 클래스
class DynamixelSdkInterface
{
public:
  static constexpr double PI = std::numbers::pi_v<double>;
  static constexpr double UNIT2RADPERSEC = 0.229 * PI * 2.0 / 60.0;
  static constexpr double RADPERSEC2UNIT = 60.0 / (0.229 * PI * 2.0);
  static constexpr double UNIT2RAD = PI * 2.0 / 4096.0;
  static constexpr double RAD2UNIT = 4096.0 / (PI * 2.0);
  static constexpr double UNIT2DEG = 360.0 / 4096.0;
  static constexpr double DEG2UNIT = 4096.0 / 360.0;

  struct State
  {
    // raw Dynamixel units (XM/XC 공통)
    std::vector<int32_t> position;  // PRESENT_POSITION (32bit signed)
    std::vector<int32_t> velocity;  // PRESENT_VELOCITY (32bit signed)
    std::vector<int16_t> current;   // PRESENT_CURRENT  (16bit signed)
  };

  explicit DynamixelSdkInterface(
      const std::string &device_name = "/dev/ttyUSB0",
      int baudrate = 57600,
      double protocol_version = 2.0);

  ~DynamixelSdkInterface();

  // 현재 상태를 한 번 읽는다 (BulkRead)
  //  - 통신 성공 여부만 true/false로 반환
  bool readOnce(State &out_state);

  // goal positions (unit) 를 BulkWrite로 한 번에 쓴다
  //  - size == jointIds().size() 이어야 함
  bool writeGoalPositions(const std::vector<int32_t> &goals_unit);

  // degree 입력으로 goal을 쓴다 (내부에서 DEG2UNIT 변환 + bound 클리핑)
  bool writeGoalPositionsDeg(const std::vector<double> &goals_deg);

  // 모든 모터 토크 ON/OFF
  void setTorqueEnabled(bool enable);

  // 현재 사용하는 Dynamixel ID 목록
  const std::vector<uint8_t> &jointIds() const { return dxl_ids_all_; }

  // bound (unit)
  const std::vector<int32_t> &lowerBounds() const { return lower_bound_unit_; }
  const std::vector<int32_t> &upperBounds() const { return upper_bound_unit_; }

private:
  void initPort();
  void initMotors();    // torque off → mode/profile 설정 → torque on
  void initBulkRead();  // PRESENT_* 3개를 GroupBulkRead에 등록

  // ===== MATLAB test.m 의 파라미터들 =====
  std::string device_name_;
  int         baudrate_;
  double      protocol_version_;

  // IDs
  std::vector<uint8_t> dxl_ids_all_;  // [1 2 3 4 5 6 7]
  std::vector<uint8_t> type_a_ids_;   // [2 4]
  std::vector<uint8_t> type_b_ids_;   // 나머지

  // operating mode
  uint8_t mode_a_; // 4
  uint8_t mode_b_; // 4

  // profile
  uint32_t prof_vel_a_; // 50
  uint32_t prof_acc_a_; // 10
  uint32_t prof_vel_b_; // 50
  uint32_t prof_acc_b_; // 10

  // joint bounds (unit)
  std::vector<int32_t> lower_bound_unit_;
  std::vector<int32_t> upper_bound_unit_;

  // control table
  uint16_t addr_torque_enable_;
  uint16_t addr_operating_mode_;
  uint16_t addr_profile_acc_;
  uint16_t addr_profile_vel_;
  uint16_t addr_goal_position_;
  uint16_t addr_present_current_;
  uint16_t addr_present_velocity_;
  uint16_t addr_present_position_;

  uint16_t len_torque_enable_;
  uint16_t len_operating_mode_;
  uint16_t len_profile_acc_;
  uint16_t len_profile_vel_;
  uint16_t len_goal_position_;
  uint16_t len_present_current_;
  uint16_t len_present_velocity_;
  uint16_t len_present_position_;
  uint16_t len_present_all_;

  uint8_t torque_enable_value_;
  uint8_t torque_disable_value_;

  // SDK objects
  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;
  std::unique_ptr<dynamixel::GroupBulkRead>  group_bulk_read_;
  std::unique_ptr<dynamixel::GroupBulkWrite> group_bulk_write_;

  rclcpp::Logger logger_;
};
