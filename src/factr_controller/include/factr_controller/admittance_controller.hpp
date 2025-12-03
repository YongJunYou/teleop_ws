#pragma once

#include <Eigen/Dense>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "factr_controller/dynamixel_sdk_interface.hpp"

class AdmittanceController
{
public:
    AdmittanceController();
    ~AdmittanceController();

    void update(const Eigen::VectorXd &external_torque,
                const DynamixelSdkInterface::State &state,
                double dt);

    // Get desired joint acceleration
    Eigen::VectorXd getDesiredJointAcceleration() const;

    // Get desired joint velocity (integrated from acceleration)
    Eigen::VectorXd getDesiredJointVelocity() const;

    // Get desired joint position (double integrated from acceleration)
    Eigen::VectorXd getDesiredJointPosition() const;

private:
    void initializeModel();
    void computeJacobian(const Eigen::VectorXd &q);
    void computeJacobianDot(const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot);

    pinocchio::Model model_;
    pinocchio::Data data_;
    bool model_ready_;
    pinocchio::FrameIndex end_effector_frame_id_;

    // Desired admittance parameters
    Eigen::MatrixXd Md_;  // Desired mass matrix (6x6 for task space)
    Eigen::MatrixXd Dd_;  // Desired damping matrix (6x6)
    Eigen::MatrixXd Kd_;  // Desired stiffness matrix (6x6)

    // Task space state
    Eigen::VectorXd x_;        // Task space position (6x1: 3 pos + 3 ori)
    Eigen::VectorXd x_dot_;    // Task space velocity (6x1)
    Eigen::VectorXd x_des_dot_dot_;  // Desired task space acceleration (6x1)

    // Joint space state
    Eigen::VectorXd q_;        // Joint position (current measured)
    Eigen::VectorXd q_dot_;    // Joint velocity (current measured)
    Eigen::VectorXd theta_d_dot_dot_;  // Desired joint acceleration

    // Desired joint states (integrated from acceleration)
    Eigen::VectorXd q_des_;        // Desired joint position (double integrated)
    Eigen::VectorXd q_dot_des_;   // Desired joint velocity (integrated)
    Eigen::VectorXd theta_d_dot_dot_prev_;  // Previous acceleration for better integration

    // Jacobian and its time derivative
    Eigen::MatrixXd J_;        // Jacobian (6x7)
    Eigen::MatrixXd J_prev_;   // Previous Jacobian for computing J_dot
    Eigen::MatrixXd J_dot_;    // Time derivative of Jacobian (6x7)
    Eigen::MatrixXd J_pinv_;   // Pseudo-inverse of Jacobian (7x6)

    // Previous state for computing derivatives
    Eigen::VectorXd q_prev_;
    Eigen::VectorXd x_prev_;
    bool first_update_;

    // Integration state
    bool integration_initialized_;

    Eigen::VectorXd last_external_torque_;
};