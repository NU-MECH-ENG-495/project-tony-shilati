#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "../../src/include/open_chain_kinematics.hpp"

TEST(RigidBodyTest, VecToso3){
    Eigen::VectorXd omega(3);
    omega << 1, 2, 3;
    Eigen::MatrixXd so3mat = rigid_body_motion::VecToso3(omega);
    Eigen::MatrixXd expected(3, 3);
    expected << 0, -3, 2,
                3, 0, -1,
                -2, 1, 0;
    ASSERT_TRUE(so3mat.isApprox(expected));
}

TEST(RigidBodyTest, so3ToVec){
    Eigen::MatrixXd so3mat(3, 3);
    so3mat << 0, -3, 2,
              3, 0, -1,
              -2, 1, 0;
    Eigen::VectorXd omega = rigid_body_motion::so3ToVec(so3mat);
    Eigen::VectorXd expected(3);
    expected << 1, 2, 3;
    ASSERT_TRUE(omega.isApprox(expected));
}

TEST(RigidBodyTest, Rodriguez_1){

    // Define angle and axis of roation
    float theta = M_PI/2;
    Eigen::VectorXd omg(3);
    omg << 1, 2, 3;

    // Evaluate the Rodrigues formula
    Eigen::MatrixXd R = rigid_body_motion::Rodriguez(omg, theta);

    // Evaluate the expected result
    Eigen::VectorXd omg_hat(3);
    omg_hat = omg.normalized();
    Eigen::MatrixXd omg_hat_so3(3, 3);
    omg_hat_so3 = rigid_body_motion::VecToso3(omg_hat);
    Eigen::MatrixXd expected = Eigen::MatrixXd::Identity(3, 3) + omg_hat_so3 * sin(theta) + omg_hat_so3 * omg_hat_so3 * (1 - cos(theta));

    ASSERT_TRUE(R.isApprox(expected));
}

TEST(RigidBodyTest, Rodriguez_2){
    Eigen::VectorXd omega(3);
    omega << 1, 2, 3;
    Eigen::MatrixXd R = rigid_body_motion::Rodriguez(omega, 0);
    Eigen::MatrixXd expected(3, 3);
    expected = Eigen::MatrixXd::Identity(3, 3);

    ASSERT_TRUE(R.isApprox(expected));
}

TEST(RigidBodyTest, RotationLogarithm_1){
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd omega = rigid_body_motion::RotationLogarithm(R);
    Eigen::VectorXd expected(3);
    expected << 0, 0, 0;
    ASSERT_TRUE(omega.isApprox(expected));
}

TEST(RigidBodyTest, RotationLogarithm_2){
    Eigen::MatrixXd R(3, 3);
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
    Eigen::VectorXd omega = rigid_body_motion::RotationLogarithm(R);
    Eigen::VectorXd expected(3);
    expected << 0, 0, M_PI/2;
    ASSERT_TRUE(omega.isApprox(expected));
}


TEST(RigidBodyTest, Adjoint){
    // Define a translationa and roation
    Eigen::VectorXd p(3); p << 1, 2, 3;
    Eigen::MatrixXd R(3, 3);
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
    Eigen::MatrixXd T(4, 4);
    T << R, p,
         0, 0, 0, 1;

    // Evaluate the adjoint function
    Eigen::MatrixXd Ad_T = rigid_body_motion::Adjoint(T);

    // Construct the expected result
    Eigen::MatrixXd expected(6, 6);
    expected << R, Eigen::MatrixXd::Zero(3, 3),
                rigid_body_motion::VecToso3(p) * R, R;

    ASSERT_TRUE(Ad_T.isApprox(expected));
}

TEST(RigidBodyTest, Matrix_Exponential){
    // Define a twist
    Eigen::VectorXd S(6);
    S << 1, 2, 3, 4, 5, 6;
    float theta = M_PI/2;

    // Evaluate the matrix exponential
    Eigen::MatrixXd exp_S_theta = rigid_body_motion::Matrix_Exponential(S, theta);

    // Construct the expected result
    Eigen::VectorXd omega(3); omega << 1, 2, 3;
    Eigen::VectorXd v(3); v << 4, 5, 6;
    Eigen::MatrixXd omega_so3 = rigid_body_motion::VecToso3(omega);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd R = rigid_body_motion::Rodriguez(omega, theta);
    Eigen::MatrixXd V = (I * theta + (1 - cos(theta)) * omega_so3 + (theta - sin(theta)) * omega_so3 * omega_so3) * v;
    Eigen::MatrixXd expected(4, 4);
    expected << R, V,
                0, 0, 0, 1;

    ASSERT_TRUE(exp_S_theta.isApprox(expected));
}