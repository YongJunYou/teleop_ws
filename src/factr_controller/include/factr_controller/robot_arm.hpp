#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "factr_controller/admittance_controller.hpp"
#include "factr_controller/external_torque_estimator.hpp"
#include "factr_controller/dynamixel_sdk_interface.hpp"

#include <Eigen/Dense>
#include <optional>

class RobotArm : public rclcpp::Node
{
public:
    RobotArm();
    void run();

private:
    rclcpp::Time last_time_;

    std::vector<int> dxl_ids_all_ = {1, 2, 3, 4, 5, 6, 7};

    std::optional<DynamixelSdkInterface> dxl_;

    ExternalTorqueEstimator estimator_;
    AdmittanceController admittance_;

    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr external_torque_pub_;

    // 디버그용 헬퍼
    void debugDynamixelState(const DynamixelSdkInterface::State &state);
    void debugExternalTorque(const Eigen::VectorXd &tau_ext);
    
    // Topic 발행 함수
    void publishJointState(const DynamixelSdkInterface::State &state);
    void publishExternalTorque(const Eigen::VectorXd &tau_ext);
};

