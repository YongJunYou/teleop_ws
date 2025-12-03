#include "factr_controller/admittance_controller.hpp"

#include <iostream>
#include <algorithm>
#include <Eigen/Geometry>

namespace
{
constexpr char kUrdfFilename[] =
    "/home/yyj/ros2_workspaces/teleop_ws/src/factr_urdf_description/urdf/factr_teleop_franka.urdf";
}

AdmittanceController::AdmittanceController()
: model_()
, data_(model_)
, model_ready_(false)
, end_effector_frame_id_(0)
, Md_(Eigen::MatrixXd::Identity(6, 6))
, Dd_(Eigen::MatrixXd::Identity(6, 6))
, Kd_(Eigen::MatrixXd::Identity(6, 6))
, x_(Eigen::VectorXd::Zero(6))
, x_dot_(Eigen::VectorXd::Zero(6))
, x_des_dot_dot_(Eigen::VectorXd::Zero(6))
, q_(Eigen::VectorXd::Zero(7))
, q_dot_(Eigen::VectorXd::Zero(7))
, theta_d_dot_dot_(Eigen::VectorXd::Zero(7))
, q_des_(Eigen::VectorXd::Zero(7))
, q_dot_des_(Eigen::VectorXd::Zero(7))
, theta_d_dot_dot_prev_(Eigen::VectorXd::Zero(7))
, J_(Eigen::MatrixXd::Zero(6, 7))
, J_prev_(Eigen::MatrixXd::Zero(6, 7))
, J_dot_(Eigen::MatrixXd::Zero(6, 7))
, J_pinv_(Eigen::MatrixXd::Zero(7, 6))
, q_prev_(Eigen::VectorXd::Zero(7))
, x_prev_(Eigen::VectorXd::Zero(6))
, first_update_(true)
, integration_initialized_(false)
, last_external_torque_(Eigen::VectorXd::Zero(7))
{
    initializeModel();

    // Set default admittance parameters (can be tuned)
    // Md: desired mass matrix
    Md_.setIdentity();
    Md_.block<3, 3>(0, 0) *= 1.0;  // translational mass
    Md_.block<3, 3>(3, 3) *= 0.1;  // rotational mass

    // Dd: desired damping matrix
    Dd_.setIdentity();
    Dd_.block<3, 3>(0, 0) *= 50.0;  // translational damping
    Dd_.block<3, 3>(3, 3) *= 5.0;   // rotational damping

    // Kd: desired stiffness matrix
    Kd_.setIdentity();
    Kd_.block<3, 3>(0, 0) *= 100.0;  // translational stiffness
    Kd_.block<3, 3>(3, 3) *= 10.0;   // rotational stiffness
}

AdmittanceController::~AdmittanceController() = default;

void AdmittanceController::initializeModel()
{
    const std::string urdf_filename{kUrdfFilename};

    try {
        pinocchio::urdf::buildModel(urdf_filename, model_);
        data_ = pinocchio::Data(model_);

        // Find end-effector frame (link_7)
        if (model_.existFrame("link_7")) {
            end_effector_frame_id_ = model_.getFrameId("link_7");
        } else {
            // If frame doesn't exist, use the last joint frame
            end_effector_frame_id_ = model_.nframes - 1;
        }

        // Resize vectors
        const int nv = model_.nv;
        q_.resize(model_.nq);
        q_dot_.resize(nv);
        q_prev_.resize(model_.nq);
        theta_d_dot_dot_.resize(nv);
        last_external_torque_.resize(nv);

        model_ready_ = true;
        std::cout << "[AdmittanceController]"
                  << "\033[32m" << " Loaded URDF SUCCESS" << "\033[0m"
                  << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "\033[31m"
                  << "[AdmittanceController] Failed to load URDF "
                  << urdf_filename << " : " << e.what()
                  << "\033[0m" << std::endl;
        model_ready_ = false;
    }
}

void AdmittanceController::computeJacobian(const Eigen::VectorXd &q)
{
    if (!model_ready_) return;

    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::framesForwardKinematics(model_, data_, q);
    pinocchio::getFrameJacobian(model_, data_, end_effector_frame_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, J_);
}

void AdmittanceController::computeJacobianDot(const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot)
{
    (void)q;
    (void)q_dot;
    if (!model_ready_ || first_update_) {
        J_dot_.setZero();
        return;
    }

    // Compute J_dot using finite difference approximation
    // J_dot ≈ (J_current - J_prev) / dt
    // More accurate: use pinocchio::computeJointJacobiansTimeVariation
    // Note: This function is not used directly, J_dot is computed in update()
    // This is kept for potential future use
}

void AdmittanceController::update(const Eigen::VectorXd &external_torque,
                                  const DynamixelSdkInterface::State &state,
                                  double dt)
{
    if (!model_ready_ || dt <= 0.0) {
        return;
    }

    last_external_torque_ = external_torque;

    const std::size_t nv = model_.nv;
    const std::size_t nmeas = std::min<std::size_t>(nv, state.position.size());
    if (nmeas == 0 || state.velocity.size() < nmeas) {
        return;
    }

    // 1. Convert state to joint space (q, q_dot)
    for (std::size_t i = 0; i < nmeas; ++i) {
        const auto idx = static_cast<Eigen::Index>(i);
        q_(idx) = static_cast<double>(state.position[i]) * DynamixelSdkInterface::UNIT2RAD;
        q_dot_(idx) = static_cast<double>(state.velocity[i]) * DynamixelSdkInterface::UNIT2RADPERSEC;
    }

    // 2. Compute forward kinematics and Jacobian
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::framesForwardKinematics(model_, data_, q_);
    
    // Get end-effector pose
    const pinocchio::SE3 &T_ee = data_.oMf[end_effector_frame_id_];
    x_.head<3>() = T_ee.translation();
    
    // Convert rotation matrix to axis-angle representation for orientation
    // R = exp([w]_×) where w is the axis-angle vector
    Eigen::Matrix3d R = T_ee.rotation();
    Eigen::AngleAxisd angle_axis(R);
    x_.tail<3>() = angle_axis.angle() * angle_axis.axis();

    // Compute Jacobian
    computeJacobian(q_);

    // 3. Compute J_dot (time derivative of Jacobian)
    if (first_update_) {
        J_dot_.setZero();
        J_prev_ = J_;
        x_prev_ = x_;
        q_prev_ = q_;
        first_update_ = false;
    } else {
        // Finite difference approximation for J_dot
        // J_dot ≈ (J_current - J_prev) / dt
        // More accurate would be: pinocchio::computeJointJacobiansTimeVariation
        J_dot_ = (J_ - J_prev_) / dt;
        J_prev_ = J_;
    }

    // 4. Compute task space velocity: x_dot = J * q_dot
    x_dot_ = J_ * q_dot_;

    // 5. Convert external torque to task space force
    // f_ext = J^{+T} * tau_ext
    // Compute pseudo-inverse of J
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const double threshold = 1e-6;
    Eigen::VectorXd singular_values = svd.singularValues();
    const int n_sv = static_cast<int>(singular_values.size());
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(n_sv, n_sv);
    for (int i = 0; i < n_sv; ++i) {
        if (singular_values(i) > threshold) {
            S_inv(i, i) = 1.0 / singular_values(i);
        }
    }
    J_pinv_ = svd.matrixV() * S_inv * svd.matrixU().transpose();

    // f_ext = J^{+T} * tau_ext
    Eigen::VectorXd f_ext = J_pinv_.transpose() * external_torque;

    // 6. Compute desired task space acceleration
    // x_des_dot_dot = Md^-1 (f_ext - Dd * x_dot + Kd * x)
    x_des_dot_dot_ = Md_.inverse() * (f_ext - Dd_ * x_dot_ + Kd_ * x_);

    // 7. Convert to desired joint acceleration
    // theta_d_dot_dot = J^+ (x_des_dot_dot - J_dot * q_dot)
    theta_d_dot_dot_ = J_pinv_ * (x_des_dot_dot_ - J_dot_ * q_dot_);

    // 8. Numerically integrate acceleration to get velocity and position
    // Use trapezoidal rule for better numerical stability
    if (!integration_initialized_) {
        // Initialize with current measured state
        q_des_ = q_;
        q_dot_des_ = q_dot_;
        theta_d_dot_dot_prev_ = theta_d_dot_dot_;
        integration_initialized_ = true;
    } else {
        // Clamp dt to reasonable range for numerical stability
        const double dt_clamped = std::max(1e-6, std::min(dt, 0.1));
        
        // Store previous velocity before updating
        Eigen::VectorXd q_dot_des_prev = q_dot_des_;
        
        // Integrate velocity: v = v_prev + a_avg * dt
        // Using trapezoidal rule: a_avg = (a_prev + a_current) / 2
        Eigen::VectorXd a_avg = 0.5 * (theta_d_dot_dot_prev_ + theta_d_dot_dot_);
        q_dot_des_ += a_avg * dt_clamped;
        
        // Integrate position: q = q_prev + v_avg * dt
        // Using trapezoidal rule: v_avg = (v_prev + v_current) / 2
        Eigen::VectorXd v_avg = 0.5 * (q_dot_des_prev + q_dot_des_);
        q_des_ += v_avg * dt_clamped;
        
        // Store current acceleration for next iteration
        theta_d_dot_dot_prev_ = theta_d_dot_dot_;
    }

    // Store current state for next iteration
    q_prev_ = q_;
    x_prev_ = x_;
}

Eigen::VectorXd AdmittanceController::getDesiredJointAcceleration() const
{
    return theta_d_dot_dot_;
}

Eigen::VectorXd AdmittanceController::getDesiredJointVelocity() const
{
    return q_dot_des_;
}

Eigen::VectorXd AdmittanceController::getDesiredJointPosition() const
{
    return q_des_;
}
