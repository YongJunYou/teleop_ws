#include "factr_controller/robot_arm.hpp"
#include <sstream>
#include <cmath>

RobotArm::RobotArm()
: Node("robot_arm"),
  estimator_(),
  admittance_()
{
    // USB 포트 및 보드레이트 파라미터 선언
    this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 4000000);
    this->declare_parameter<double>("protocol_version", 2.0);

    // 파라미터 읽기
    std::string device_name = this->get_parameter("device_name").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    double protocol_version = this->get_parameter("protocol_version").as_double();

    // DynamixelSdkInterface를 파라미터 값으로 초기화
    dxl_.emplace(device_name, baudrate, protocol_version);

    // ROS2 Publisher 초기화
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    external_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "external_torque", 10);

    RCLCPP_INFO(this->get_logger(), "RobotArm created.");
    RCLCPP_INFO(this->get_logger(), "Dynamixel device: %s @ %d baud", 
                device_name.c_str(), baudrate);
    RCLCPP_INFO(this->get_logger(), "Publishing joint states to topic: joint_states");
    RCLCPP_INFO(this->get_logger(), "Publishing external torque to topic: external_torque");
}

void RobotArm::run()
{
    RCLCPP_INFO(this->get_logger(), "Starting control loop...");

    last_time_ = this->now();
    const double control_period = 0.005;  // 5ms
    rclcpp::Rate rate(1.0 / control_period);  // 200 Hz

    DynamixelSdkInterface::State dxl_state;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (!std::isfinite(dt) || dt <= 0.0) {
            dt = control_period;
        }

        // 1. state 읽기
        if (!dxl_.has_value() || !dxl_->readOnce(dxl_state))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read Dynamixel state");
            continue;
        }
        
        // ROS2 topic으로 발행
        publishJointState(dxl_state);
        
        // 디버그 출력
        //debugDynamixelState(dxl_state);
        
        // 2. 추정기
        estimator_.update(dxl_state, dt);
        const Eigen::VectorXd tau_ext = estimator_.getExternalTorque();
        
        // ROS2 topic으로 발행
        publishExternalTorque(tau_ext);
        
        // 디버그 출력 (너무 빨라 안 읽힘)
        //debugExternalTorque(tau_ext);
        
        // 3. 제어기
       //  admittance_.update(tau_ext, dxl_state, dt);
        
        // 4. 제어기 출력 가져오기
        // const Eigen::VectorXd q_d = admittance_.getDesiredJointPosition();
        
        // 5. Dynamixel에 쓰기 (radian을 unit으로 변환)
        // if (q_d.size() == static_cast<Eigen::Index>(dxl_state.position.size())) {
        //     std::vector<int32_t> q_d_unit(q_d.size());
        //     for (int i = 0; i < q_d.size(); ++i) {
        //         // RAD2UNIT = 4096.0 / (PI * 2.0)
        //         // std::round()를 사용하여 반올림
        //         q_d_unit[i] = static_cast<int32_t>(std::round(q_d(i) * DynamixelSdkInterface::RAD2UNIT));
        //     }
            
        //     if (!dxl_->writeGoalPositions(q_d_unit)) {
        //         RCLCPP_WARN_THROTTLE(
        //             this->get_logger(),
        //             *this->get_clock(),
        //             1.0,  // 1초마다 출력
        //             "Failed to write goal positions to Dynamixel");
        //     }
        // }
        
        rate.sleep();  // 여기서 다음 주기까지 블록
    }
}

void RobotArm::debugDynamixelState(const DynamixelSdkInterface::State &state)
{
    const std::size_t n = state.position.size();
    if (n == 0) return;

    const double UNIT2DEG = DynamixelSdkInterface::UNIT2DEG;
    
    // Position (deg)
    Eigen::VectorXd q_deg(static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i)
    {
        q_deg(static_cast<int>(i)) =
            static_cast<double>(state.position[i]) * UNIT2DEG;
    }

    // Velocity (raw units, typically 0.229 rev/min per unit for XM/XC series)
    Eigen::VectorXd v_raw(static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i)
    {
        v_raw(static_cast<int>(i)) = static_cast<double>(state.velocity[i]);
    }

    // Current (raw units, typically 2.69 mA per unit for XM430, 1.0 mA per unit for XC330)
    Eigen::VectorXd i_raw(static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i)
    {
        i_raw(static_cast<int>(i)) = static_cast<double>(state.current[i]);
    }

    // 출력
    std::stringstream ss_pos, ss_vel, ss_cur;
    ss_pos << q_deg.transpose();
    ss_vel << v_raw.transpose();
    ss_cur << i_raw.transpose();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5,  // 5ms (0.5초마다 출력)
        "Dynamixel State - Position [deg]: %s | Velocity [raw]: %s | Current [raw]: %s",
        ss_pos.str().c_str(),
        ss_vel.str().c_str(),
        ss_cur.str().c_str());
}

void RobotArm::debugExternalTorque(const Eigen::VectorXd &tau_ext)
{
    if (tau_ext.size() == 0) return;

    std::stringstream ss;
    ss << tau_ext.transpose();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5,  // 0.5초마다 출력
        "External Torque [Nm]: %s",
        ss.str().c_str());
}

void RobotArm::publishJointState(const DynamixelSdkInterface::State &state)
{
    if (!joint_state_pub_) return;

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    
    const std::size_t n = state.position.size();
    if (n == 0) return;

    // Joint names
    msg.name.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        msg.name[i] = "joint_" + std::to_string(i + 1);
    }

    // Position (radian)
    msg.position.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        msg.position[i] = static_cast<double>(state.position[i]) * 
                          DynamixelSdkInterface::UNIT2RAD;
    }

    // Velocity (rad/s)
    msg.velocity.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        msg.velocity[i] = static_cast<double>(state.velocity[i]) * 
                          DynamixelSdkInterface::UNIT2RADPERSEC;
    }

    // Effort (current를 토크로 변환 - 단순화된 변환)
    // 실제로는 ExternalTorqueEstimator에서 계산한 값을 사용하는 것이 더 정확합니다
    msg.effort.resize(n);
    for (std::size_t i = 0; i < n && i < state.current.size(); ++i) {
        // Current를 토크로 변환 (간단한 근사값, 실제로는 모터 스펙에 따라 다름)
        const double current_amp = static_cast<double>(state.current[i]) * 0.00269; // 예시 값
        msg.effort[i] = current_amp * 0.1; // 예시 변환 계수 (실제 값으로 교체 필요)
    }

    joint_state_pub_->publish(msg);
}

void RobotArm::publishExternalTorque(const Eigen::VectorXd &tau_ext)
{
    if (!external_torque_pub_ || tau_ext.size() == 0) return;

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(tau_ext.size());
    
    for (int i = 0; i < tau_ext.size(); ++i) {
        msg.data[i] = tau_ext(i);
    }

    external_torque_pub_->publish(msg);
}

