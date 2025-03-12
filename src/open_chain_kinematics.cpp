#include <iostream>
#include <Eigen/Dense>
#include "include/open_chain_kinematics.hpp"
#include <math.h>

namespace rigid_body_motion {
    Eigen::MatrixXd VecToso3(const Eigen::VectorXd &omega){
        // Converts a 3-vector to its s03 representation

    }

    Eigen::VectorXd so3ToVec(const Eigen::MatrixXd &so3mat){
        // Converts an so3 matrix to a vector
        
    }

    Eigen::MatrixXd Rodriguez(const Eigen::VectorXd &omega, const float &theta){
        // Implements Rodriguez formula to calculate a rotation matrix from an axis and angle

    }

    Eigen::MatrixXd Matrix_Exponential(const Eigen::VectorXd &S, const float &theta){
        // Calculates the matrix exponential of a rigid body motion

    }

    Eigen::MatrixXd Matrix_Logarithm(const Eigen::MatrixXd &T){
        // Calculates the matrix logarithm of a rigid body motion

    }

}

namespace open_chain_kinematics {

    Eigen::VectorXd FKin_Space(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> S_list, const Eigen::VectorXd theta_list) {
        // Forward kinematics in the space frame

    }

    Eigen::VectorXd FKin_Body(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> B_list, const Eigen::VectorXd theta_list) {
        // Forward kinematics in the body frame
        
    }

} // namespace open_chain_kinematics