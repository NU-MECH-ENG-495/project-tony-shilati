#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "../../src/include/open_chain_kinematics.hpp"


/**
 * @brief Test the forward kinematics in space frame.
 */
TEST(KinematicsTest, FKin_Space) {
    std::vector<float> l = {1, 1, 1};

    Eigen::VectorXd theta_list(3);
    theta_list << 0, 0, 0;


    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(4, 4);
    M(0, 3) = l[0] + l[1] + l[2];

    std::vector<Eigen::VectorXd> S_list;
    Eigen::VectorXd S1(6);  S1 << 0, 0, 1, 0, 0, 0;             S_list.push_back(S1);
    Eigen::VectorXd S2(6);  S2 << 0, 0, 1, 0, -l[0], 0;         S_list.push_back(S2);
    Eigen::VectorXd S3(6);  S3 << 0, 0, 1, 0, -l[0]-l[1], 0;    S_list.push_back(S3);


    Eigen::MatrixXd T = open_chain_kinematics::FKin_Space(M, S_list, theta_list);
    ASSERT_TRUE(T.isApprox(M));
}

/**
 * @brief Test the forward kinematics in body frame.
 */
TEST(KinematicsTest, FKin_Body) {
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(4, 4);
    std::vector<Eigen::VectorXd> B_list = {Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd theta_list(2);
    theta_list << 0, 0;

    Eigen::MatrixXd T = open_chain_kinematics::FKin_Body(M, B_list, theta_list);
    ASSERT_TRUE(T.isApprox(M));
}

/**
 * @brief Test the inverse kinematics in space frame.
 */
TEST(KinematicsTest, IKin_Space) {
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(4, 4);
    std::vector<Eigen::VectorXd> S_list = {Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)};
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);

    Eigen::VectorXd theta_list = open_chain_kinematics::IKin_Space(M, S_list, T);
    ASSERT_TRUE(theta_list.isZero());
}

/**
 * @brief Test the inverse kinematics in body frame.
 */
TEST(KinematicsTest, IKin_Body) {
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(4, 4);
    std::vector<Eigen::VectorXd> B_list = {Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)};
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);

    Eigen::VectorXd theta_list = open_chain_kinematics::IKin_Body(M, B_list, T);
    ASSERT_TRUE(theta_list.isZero());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
