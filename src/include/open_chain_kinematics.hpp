#ifndef OPEN_CHAIN_KINEMATICS_HPP
#define OPEN_CHAIN_KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>

namespace rigid_body_motion {
    Eigen::MatrixXd VecToso3(const Eigen::VectorXd &omega);
    Eigen::VectorXd so3ToVec(const Eigen::MatrixXd &so3mat);
    Eigen::MatrixXd Rodriguez(const Eigen::VectorXd &omega, const float &theta);
    Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &T);
    Eigen::MatrixXd Matrix_Exponential(const Eigen::VectorXd &S, const float &theta);
    Eigen::MatrixXd Matrix_Logarithm(const Eigen::MatrixXd &T);

}

namespace open_chain_kinematics {
    Eigen::VectorXd FKin_Space(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> S_list, const Eigen::VectorXd theta_list);
    Eigen::VectorXd FKin_Body(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> B_list, const Eigen::VectorXd theta_list);

} // namespace open_chain_kinematics

#endif // OPEN_CHAIN_KINEMATICS_HPP