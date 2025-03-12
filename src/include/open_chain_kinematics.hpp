#ifndef OPEN_CHAIN_KINEMATICS_HPP
#define OPEN_CHAIN_KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>

namespace open_chain_kinematics {

    Eigen::VectorXd FKin_Space(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> S_list, const std::vector<float> theta_list);
    Eigen::VectorXd FKin_Body(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> B_list, const std::vector<float> theta_list);

} // namespace open_chain_kinematics

#endif // OPEN_CHAIN_KINEMATICS_HPP