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
, x0_(Eigen::VectorXd::Zero(6))
, q_(Eigen::VectorXd::Zero(7))
, q_dot_(Eigen::VectorXd::Zero(7))
, q_des_(Eigen::VectorXd::Zero(7))
, q_dot_des_(Eigen::VectorXd::Zero(7))
, q_dot_dot_des_(Eigen::VectorXd::Zero(7))
, J_(Eigen::MatrixXd::Zero(6, 7))
, J_prev_(Eigen::MatrixXd::Zero(6, 7))
, J_dot_(Eigen::MatrixXd::Zero(6, 7))
, J_transpose_pinv_(Eigen::MatrixXd::Zero(6, 7))
, J_pinv_(Eigen::MatrixXd::Zero(7, 6))
, first_update_(true)
, last_external_torque_(Eigen::VectorXd::Zero(7))
, f_ext_(Eigen::VectorXd::Zero(6))
{
    initializeModel();

    // Set default admittance parameters (can be tuned)
    // Md: desired mass matrix
    Md_.setIdentity();
    Md_.block<3, 3>(0, 0) *= 5.0;  // translational mass
    Md_.block<3, 3>(3, 3) *= 100.0;  // rotational mass

    // Dd: desired damping matrix
    Dd_.setIdentity();
    Dd_.block<3, 3>(0, 0) *= 40.0;  // translational damping
    Dd_.block<3, 3>(3, 3) *= 40.0;   // rotational damping

    // Kd: desired stiffness matrix
    Kd_.setIdentity();
    Kd_.block<3, 3>(0, 0) *= 500.0;  // translational stiffness
    Kd_.block<3, 3>(3, 3) *= 500.0;   // rotational stiffness

    // Reference position x0_ will be set via setReferencePosition() from ROS parameters
    // or initialized from current end-effector position on first update
    x0_.setZero(6);
    // Position part (indices 0, 1, 2) and orientation part (indices 3, 4, 5) remain zero
    // until explicitly set via setReferencePosition() or initialized from current pose
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
        q_dot_dot_des_.resize(nv);
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
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_); /// check
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
    } else {
        // Finite difference approximation for J_dot
        // J_dot ≈ (J_current - J_prev) / dt
        // More accurate would be: pinocchio::computeJointJacobiansTimeVariation
        J_dot_ = (J_ - J_prev_) / dt;   //------------------------------------------------> 범인 유력
        J_prev_ = J_;
    }

    // 4. Compute task space velocity: x_dot = J * q_dot
    x_dot_ = J_ * q_dot_;

    // 5. Convert external torque to task space force
    // f_ext = J^{+T} * tau_ext
    // Compute pseudo-inverse of J
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J_,
        Eigen::ComputeThinU | Eigen::ComputeThinV);

    const double threshold = 1e-6;
    Eigen::VectorXd singular_values = svd.singularValues();
    const int n_sv = static_cast<int>(singular_values.size());

    Eigen::MatrixXd S_inv =
        Eigen::MatrixXd::Zero(n_sv, n_sv);

    for (int i = 0; i < n_sv; ++i)
    {
        if (singular_values(i) > threshold)
        {
            S_inv(i, i) = 1.0 / singular_values(i);
        }
    }

    J_pinv_ =
        svd.matrixV() *
        S_inv *
        svd.matrixU().transpose();

    f_ext_ = J_pinv_.transpose() * external_torque;

    // 6. Compute desired task space acceleration
    x_des_dot_dot_ = Md_.inverse() * (f_ext_ - Dd_ * x_dot_ - Kd_ * (x_ - x0_)); // x를 구해서  x->q 로 변환하면 된다.

    // 7. Convert to desired joint acceleration
    q_dot_dot_des_ = J_pinv_ * (x_des_dot_dot_ - J_dot_ * q_dot_);

    // 8. Numerical integration
    if (first_update_)
    {
        q_des_ = q_;
        q_dot_des_ = Eigen::VectorXd::Zero(7);
        // Eigen::VectorXd q_dot_des_prev = q_dot_des_;
        // Eigen::VectorXd q_des_prev = q_des_;
        first_update_ = false;
    }
    else
    {
        Eigen::VectorXd q_dot_des_prev = q_dot_des_;
        Eigen::VectorXd q_des_prev = q_des_;

        q_dot_des_ =
            q_dot_des_prev + q_dot_dot_des_ * dt;
        q_des_ =
            q_des_prev + q_dot_des_ * dt;
    }
}

Eigen::VectorXd AdmittanceController::getDesiredJointAcceleration() const
{
    return q_dot_dot_des_;
}

Eigen::VectorXd AdmittanceController::getDesiredJointVelocity() const
{
    return q_dot_des_;
}

Eigen::VectorXd AdmittanceController::getDesiredJointPosition() const
{
    return q_des_;
}

void AdmittanceController::setReferencePosition(double x, double y, double z)
{
    x0_(0) = x;
    x0_(1) = y;
    x0_(2) = z;
}

Eigen::VectorXd AdmittanceController::getExternalForce() const
{
    return f_ext_;
}

Eigen::MatrixXd AdmittanceController::getJacobian() const
{
    return J_;
}

Eigen::Vector3d AdmittanceController::getEndEffectorPosition(
    const DynamixelSdkInterface::State &state) const
{
    if (!model_ready_)
    {
        return Eigen::Vector3d::Zero();
    }

    const std::size_t nv = model_.nv;
    const std::size_t nmeas =
        std::min<std::size_t>(nv, state.position.size());

    if (nmeas == 0)
    {
        return Eigen::Vector3d::Zero();
    }

    Eigen::VectorXd q(model_.nq);

    for (std::size_t i = 0; i < nmeas; ++i)
    {
        const auto idx = static_cast<Eigen::Index>(i);
        q(idx) =
            static_cast<double>(state.position[i]) *
            DynamixelSdkInterface::UNIT2RAD;
    }

    pinocchio::Data data_temp(model_);
    pinocchio::forwardKinematics(model_, data_temp, q);
    pinocchio::framesForwardKinematics(model_, data_temp, q);

    const pinocchio::SE3 &T_ee =
        data_temp.oMf[end_effector_frame_id_];

    return T_ee.translation();
}



















































































