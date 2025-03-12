#include <iostream>
#include <Eigen/Dense>
#include "include/open_chain_kinematics.hpp"

namespace open_chain_kinematics {

    Eigen::VectorXd FKin_Space(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> S_list, const std::vector<float> theta_list) {
        // Function definition for FKin_Space
        // ...implementation code...
    }

    Eigen::VectorXd FKin_Body(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> B_list, const std::vector<float> theta_list) {
        // Function definition for FKin_Body
        // ...implementation code...
    }

} // namespace open_chain_kinematics