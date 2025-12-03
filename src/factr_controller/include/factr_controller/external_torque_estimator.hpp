#pragma once

#include <Eigen/Dense>
#include <string>
#include <numbers>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "factr_controller/dynamixel_sdk_interface.hpp"

class ExternalTorqueEstimator
{
public:
    ExternalTorqueEstimator();
    ~ExternalTorqueEstimator();

    // robot_arm.cpp에서 쓰는 함수
    // dt는 [s], 기본 1kHz 제어 주기 가정 (원하면 로봇 노드에서 직접 넣어줘도 됨)
    void update(const DynamixelSdkInterface::State &state, double dt);

    Eigen::VectorXd getExternalTorque() const;

private:
    pinocchio::Model model_;
    pinocchio::Data  data_;
    bool model_ready_;
    Eigen::VectorXd tau_ext_hat_;   // DOB가 추정한 외력 토크 \hat{\tau}_{ext}

    // === DOB 내부 상태 ===
    // q_dob = [x1; x2],  p_dob = [y1; y2], 각 블록 길이 = nv
    Eigen::VectorXd q_dob_;   // size = 2*nv
    Eigen::VectorXd p_dob_;   // size = 2*nv
    Eigen::VectorXd eps_dob_; // diag{ eps_dob,i }, size = nv

    // a0 = 1*I, a1 = 2*I 를 스칼라로
    double a0_scalar_;
    double a1_scalar_;

    // 전류 → 토크 변환 계수 (joint torque = k * current)
    // 실제 값은 Dynamixel 스펙 보고 사용자가 튜닝 필요
    Eigen::VectorXd unit_to_current_;

    void initializeModel();
    double currentToTorque(std::size_t joint_index, double current_amp) const;
};
