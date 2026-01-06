#include "factr_controller/external_torque_estimator.hpp"

#include <iostream>
#include <numbers>
#include <algorithm>

namespace
{
constexpr char kUrdfFilename[] =
    "/home/yyj/ros2_workspaces/teleop_ws/src/factr_urdf_description/urdf/factr_teleop_franka.urdf";
}

ExternalTorqueEstimator::ExternalTorqueEstimator()
: model_()
, data_(model_)
, model_ready_(false)
, tau_ext_hat_() // 7x1
, q_dob_() // 14x1
, p_dob_() // 14x1
, eps_dob_() // 7x1
, unit_to_current_()
{
    initializeModel();
    if (model_ready_) {
        const std::size_t nv = model_.nv;

        // 외력 토크 벡터
        tau_ext_hat_.setZero(static_cast<Eigen::Index>(nv));

        // filtered state 벡터
        q_dob_.setZero(static_cast<Eigen::Index>(2 * nv));
        p_dob_.setZero(static_cast<Eigen::Index>(2 * nv));

        // eps_dob,i ∈ [0.1, 0.9] 튜닝중 0.8 -> 0.2
        eps_dob_.setConstant(static_cast<Eigen::Index>(nv), 0.2);

        // 전류 → 실제 전류 (모터 스펙 적용) [unit -> A]
        unit_to_current_.resize(static_cast<Eigen::Index>(nv));
        unit_to_current_
            << 1.0e-3,  // ID 1
               2.69e-3, // ID 2 (XM430)
               1.0e-3,  // ID 3
               2.69e-3, // ID 4 (XM430)
               1.0e-3,  // ID 5
               1.0e-3,  // ID 6
               1.0e-3;  // ID 7
    }
    a0_scalar_   = 1.0;
    a1_scalar_   = 2.0;
}

ExternalTorqueEstimator::~ExternalTorqueEstimator() = default;

void ExternalTorqueEstimator::initializeModel()
{
    const std::string urdf_filename{kUrdfFilename};

    try {
        pinocchio::urdf::buildModel(urdf_filename, model_);
        data_ = pinocchio::Data(model_);
        tau_ext_hat_.resize(model_.nv);
        tau_ext_hat_.setZero();

        // Find end-effector frame (link_7)
        if (model_.existFrame("link_7")) {
            end_effector_frame_id_ = model_.getFrameId("link_7");
        } else {
            // If frame doesn't exist, use the last joint frame
            end_effector_frame_id_ = model_.nframes - 1;
        }

        model_ready_ = true;
        std::cout << "[ExternalTorqueEstimator]"
                  << "\033[32m" << " Loaded URDF SUCCESS" << "\033[0m"
                  << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "\033[31m"
                  << "[ExternalTorqueEstimator] Failed to load URDF "
                  << urdf_filename << " : " << e.what()
                  << "\033[0m" << std::endl;
        model_ready_ = false;
    }
}

void ExternalTorqueEstimator::update(const DynamixelSdkInterface::State &state,
                                     double dt)
{
    if (!model_ready_) {
        return;
    }

    const std::size_t nv  = model_.nv;
    const std::size_t nmeas = std::min<std::size_t>(nv, state.position.size());
    if (nmeas == 0 || state.velocity.size() < nmeas) {
        return;
    }

    // 1. state (q, v, τ(form current)) 읽어와서 , q,v로 M(q) C(q,v) G(q) 계산
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(nv));  // === τ(form current) ===

    if (!state.current.empty()) {
        const std::size_t ncurr =
            std::min<std::size_t>(nv, state.current.size());
        for (std::size_t i = 0; i < ncurr; ++i) {
            const auto idx = static_cast<Eigen::Index>(i);
            const double current_raw = static_cast<double>(state.current[i]);
            const double current_amp = unit_to_current_(idx) * current_raw;
            tau(idx) = currentToTorque(static_cast<std::size_t>(i), current_amp);
        }
    }
    for (std::size_t i = 0; i < nmeas; ++i) {
        const auto idx = static_cast<Eigen::Index>(i);
        q(idx) = static_cast<double>(state.position[i]) * DynamixelSdkInterface::UNIT2RAD;
        v(idx) = static_cast<double>(state.velocity[i]) * DynamixelSdkInterface::UNIT2RADPERSEC;
    }

    pinocchio::computeAllTerms(model_, data_, q, v);
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose();
    const Eigen::MatrixXd &M   = data_.M;   // nv x nv
    const Eigen::VectorXd &nle = data_.nle; // C(q,v)v + g(q), size nv

    // 2. DOB용 filtered state 정의 q,p
    const Eigen::Index nv_e = static_cast<Eigen::Index>(nv);

    Eigen::VectorXd q_filtered     = q_dob_.segment(0,       nv_e);   // 벡터상단
    Eigen::VectorXd q_dot_filtered = q_dob_.segment(nv_e,    nv_e);   // 벡터하단
    Eigen::VectorXd p_filtered     = p_dob_.segment(0,       nv_e);
    Eigen::VectorXd p_dot_filtered = p_dob_.segment(nv_e,    nv_e);

    Eigen::VectorXd inv_eps  = eps_dob_.cwiseInverse(); // ε^-1
    Eigen::VectorXd inv_eps2 = inv_eps.cwiseProduct(inv_eps); // ε^-2

    Eigen::VectorXd Minv_tau = M.ldlt().solve(tau); // M^{-1}(q) * τ

    // ========================================================================================

    Eigen::VectorXd dq_filtered_dt(nv_e);
    Eigen::VectorXd dq_dot_filtered_dt(nv_e);
    Eigen::VectorXd dp_filtered_dt(nv_e);
    Eigen::VectorXd dp_dot_filtered_dt(nv_e);

    dq_filtered_dt = q_dot_filtered;
    dp_filtered_dt = p_dot_filtered;

    for (Eigen::Index i = 0; i < nv_e; ++i) {
        const double ie2 = inv_eps2(i);
        const double ie  = inv_eps(i);

        dq_dot_filtered_dt(i) = -ie2 * a0_scalar_ * q_filtered(i)
                                - ie  * a1_scalar_ * q_dot_filtered(i)
                                + ie2 * a0_scalar_ * q(i);

        dp_dot_filtered_dt(i) = -ie2 * a0_scalar_ * p_filtered(i)
                                - ie  * a1_scalar_ * p_dot_filtered(i)
                                + ie2 * a0_scalar_ * Minv_tau(i);
    }

    // === 외력 토크 추정식 ===
    // \hat{\tau}_{ext} = -M(q) * ( p_filtered - dq_dot_filtered_dt + nle )
    Eigen::VectorXd inside = p_filtered - dq_dot_filtered_dt + nle; // nv
    tau_ext_hat_ = -M * inside;                                        // nv

    // === DOB 상태 적분 (Euler) ===
    q_filtered.noalias()     += dt * dq_filtered_dt;
    q_dot_filtered.noalias() += dt * dq_dot_filtered_dt;
    p_filtered.noalias()     += dt * dp_filtered_dt;
    p_dot_filtered.noalias() += dt * dp_dot_filtered_dt;

    // 다시 q_dob_, p_dob_에 반영
    q_dob_.segment(0,    nv_e) = q_filtered;
    q_dob_.segment(nv_e, nv_e) = q_dot_filtered;
    p_dob_.segment(0,    nv_e) = p_filtered;
    p_dob_.segment(nv_e, nv_e) = p_dot_filtered;

    // // 디버그 출력 (원하면 나중에 RCLCPP로 바꿔도 됨)
    // std::cout << "[ExternalTorqueEstimator] tau_ext^hat = "
    //           << tau_ext_hat_.transpose() << std::endl;
}

Eigen::VectorXd ExternalTorqueEstimator::getExternalTorque() const
{
    return tau_ext_hat_;
}

Eigen::Vector3d ExternalTorqueEstimator::getEndEffectorPosition(const DynamixelSdkInterface::State &state) const
{
    if (!model_ready_) {
        return Eigen::Vector3d::Zero();
    }

    const std::size_t nv = model_.nv;
    const std::size_t nmeas = std::min<std::size_t>(nv, state.position.size());
    if (nmeas == 0) {
        return Eigen::Vector3d::Zero();
    }

    // Convert joint positions from units to radians
    Eigen::VectorXd q(model_.nq);
    for (std::size_t i = 0; i < nmeas; ++i) {
        const auto idx = static_cast<Eigen::Index>(i);
        q(idx) = static_cast<double>(state.position[i]) * DynamixelSdkInterface::UNIT2RAD;
    }

    // Compute forward kinematics
    pinocchio::Data data_temp(model_);
    pinocchio::forwardKinematics(model_, data_temp, q);
    pinocchio::framesForwardKinematics(model_, data_temp, q); /// check

    // Get end-effector position
    const pinocchio::SE3 &T_ee = data_temp.oMf[end_effector_frame_id_];
    return T_ee.translation();
}

double ExternalTorqueEstimator::currentToTorque(std::size_t joint_index,
                                                double current_amp) const // [A -> Nm] 검증됨
{
    const std::size_t joint_id = joint_index + 1;
    const bool is_xm430 = (joint_id == 2) || (joint_id == 4);
    const double current_amp_abs = std::abs(current_amp);   // 전류 크기만 사용
    double sign;

    if (current_amp >= 0.0) {
        sign = 1.0;
    } else {
        sign = -1.0;
    }

    if (is_xm430) {
        if (current_amp_abs <= 0.2) {
            return 0.0;
        }
        const double y =
            -0.2081222 + 1.396633 * current_amp_abs - 0.1585546 * current_amp_abs * current_amp_abs;
        return sign * std::max(0.0, y);
    } else { // XC330
        if (current_amp_abs <= 0.15) {
            return 0.0;
        }
        const double y =
            -0.1439461 + 1.4 * current_amp_abs - 0.4648192 * current_amp_abs * current_amp_abs;
        return sign * std::max(0.0, y);
    }
}
