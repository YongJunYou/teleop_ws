#include "factr_controller/robot_arm.hpp"
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

RobotArm::RobotArm()
: Node("robot_arm"),
  estimator_(),
  admittance_()
{
    // USB 포트 및 보드레이트 파라미터 선언
    this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 4000000);
    this->declare_parameter<double>("protocol_version", 2.0);

    // 초기 관절 위치 파라미터 선언 (YAML 파일에서 로드)
    std::vector<double> default_initial_positions = {180.0, 180.0, 180.0, 270.0, 180.0, 180.0, 180.0};
    this->declare_parameter<std::vector<double>>("initial_joint_positions", default_initial_positions);

    // Reference position 파라미터 선언 (YAML 파일에서 로드)
    this->declare_parameter<double>("reference_position.x", 0.06);
    this->declare_parameter<double>("reference_position.y", 0.0);
    this->declare_parameter<double>("reference_position.z", 0.2);

    // 파라미터 읽기
    std::string device_name = this->get_parameter("device_name").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    double protocol_version = this->get_parameter("protocol_version").as_double();

    // Reference position 파라미터 읽기 및 설정
    double ref_x = this->get_parameter("reference_position.x").as_double();
    double ref_y = this->get_parameter("reference_position.y").as_double();
    double ref_z = this->get_parameter("reference_position.z").as_double();
    admittance_.setReferencePosition(ref_x, ref_y, ref_z);

    // DynamixelSdkInterface를 파라미터 값으로 초기화
    dxl_.emplace(device_name, baudrate, protocol_version);

    // ROS2 Publisher 초기화
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    external_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "external_torque", 10);
    external_force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "external_force", 10);
    jacobian_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "jacobian", 10);
    end_effector_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "end_effector_position", 10);

    RCLCPP_INFO(this->get_logger(), "RobotArm created.");
    RCLCPP_INFO(this->get_logger(), "Dynamixel device: %s @ %d baud", 
                device_name.c_str(), baudrate);
    RCLCPP_INFO(this->get_logger(), "Publishing joint states to topic: joint_states");
    RCLCPP_INFO(this->get_logger(), "Publishing external torque to topic: external_torque");
    RCLCPP_INFO(this->get_logger(), "Publishing external force to topic: external_force");
    RCLCPP_INFO(this->get_logger(), "Publishing Jacobian to topic: jacobian");
    RCLCPP_INFO(this->get_logger(), "Publishing end-effector position to topic: end_effector_position");
}

void RobotArm::run()
{
    RCLCPP_INFO(this->get_logger(), "Moving to initial position...");
    moveToInitialPosition();

    RCLCPP_INFO(this->get_logger(), "Waiting 10 seconds before starting control loop...");
    std::this_thread::sleep_for(std::chrono::seconds(10));
    RCLCPP_INFO(this->get_logger(), "Starting control loop...");

    last_time_ = this->now();
    const double control_period = 0.005;  // 5ms
    rclcpp::Rate rate(1.0 / control_period);  // 200 Hz

    DynamixelSdkInterface::State dxl_state;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());

        // rclcpp::Time now = this->now();
        // double dt = (now - last_time_).seconds();
        // last_time_ = now;

        // if (first_loop) {
        //     first_loop = false;
        //     rate.sleep();
        //     continue; // 첫 루프는 dt 계산을 위한 기준점만 잡고 넘어감
        // }

        double dt = control_period;

        // 1. state 읽기
        if (!dxl_.has_value() || !dxl_->readOnce(dxl_state))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read Dynamixel state");
            continue;
        }

        // ROS2 topic으로 발행
        publishJointState(dxl_state);
        publishEndEffectorPosition(dxl_state);

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
        admittance_.update(tau_ext, dxl_state, dt);

        // External force 발행
        const Eigen::VectorXd f_ext = admittance_.getExternalForce();
        publishExternalForce(f_ext);

        // Jacobian 발행
        const Eigen::MatrixXd J = admittance_.getJacobian();
        publishJacobian(J);

        // 4. 제어기 출력 가져오기
        const Eigen::VectorXd q_d = admittance_.getDesiredJointPosition();

        // 5. Dynamixel에 쓰기 (radian을 degree로 변환하여 writeGoalPositionsDeg 사용)
        if (q_d.size() == static_cast<Eigen::Index>(dxl_state.position.size())) {
            std::vector<double> q_d_deg(q_d.size());
            for (int i = 0; i < q_d.size(); ++i) {
                // radian을 degree로 변환
                q_d_deg[i] = q_d(i) * 180.0 / DynamixelSdkInterface::PI;
            }

            if (!dxl_->writeGoalPositionsDeg(q_d_deg)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1.0,  // 1초마다 출력
                    "Failed to write goal positions to Dynamixel");
            }
        }

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

void RobotArm::publishEndEffectorPosition(const DynamixelSdkInterface::State &state)
{
    if (!end_effector_position_pub_) return;

    // external_torque_estimator를 통해 엔드 이펙터 위치 계산
    Eigen::Vector3d position = estimator_.getEndEffectorPosition(state);

    auto msg = geometry_msgs::msg::PointStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";  // 또는 적절한 frame 이름

    // Position
    msg.point.x = position(0);
    msg.point.y = position(1);
    msg.point.z = position(2);

    end_effector_position_pub_->publish(msg);
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

    // Position (degree)
    msg.position.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        msg.position[i] = static_cast<double>(state.position[i]) *
                          DynamixelSdkInterface::UNIT2DEG;
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

void RobotArm::publishExternalForce(const Eigen::VectorXd &f_ext)
{
    if (!external_force_pub_ || f_ext.size() == 0) return;

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(f_ext.size());

    for (int i = 0; i < f_ext.size(); ++i) {
        msg.data[i] = f_ext(i);
    }

    external_force_pub_->publish(msg);
}

void RobotArm::publishJacobian(const Eigen::MatrixXd &J)
{
    if (!jacobian_pub_ || J.rows() == 0 || J.cols() == 0) return;

    auto msg = std_msgs::msg::Float64MultiArray();

    // 행렬 크기 정보 설정 (row-major 순서)
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = static_cast<size_t>(J.rows());
    msg.layout.dim[0].stride = static_cast<size_t>(J.rows() * J.cols());
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = static_cast<size_t>(J.cols());
    msg.layout.dim[1].stride = static_cast<size_t>(J.cols());

    // 행렬을 row-major 순서로 1차원 배열로 변환
    msg.data.resize(static_cast<size_t>(J.rows() * J.cols()));
    for (int i = 0; i < J.rows(); ++i) {
        for (int j = 0; j < J.cols(); ++j) {
            msg.data[static_cast<size_t>(i * J.cols() + j)] = J(i, j);
        }
    }

    jacobian_pub_->publish(msg);
}

void RobotArm::moveToInitialPosition()
{
    if (!dxl_.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "Dynamixel interface not initialized");
        return;
    }

    // 각 관절의 초기 위치 (degree 단위) - YAML 파일에서 로드
    std::vector<double> initial_joint_positions_deg =
        this->get_parameter("initial_joint_positions").as_double_array();

    if (initial_joint_positions_deg.size() != dxl_ids_all_.size()) {
        RCLCPP_ERROR(this->get_logger(),
                    "Initial position size mismatch: expected %zu, got %zu",
                    dxl_ids_all_.size(), initial_joint_positions_deg.size());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending robot to initial position...");
    if (dxl_->writeGoalPositionsDeg(initial_joint_positions_deg)) {
        RCLCPP_INFO(this->get_logger(), "Initial position command sent successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send initial position command");
    }
}
