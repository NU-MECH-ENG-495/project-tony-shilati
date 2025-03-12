#include <iostream>
#include <Eigen/Dense>
#include "include/open_chain_kinematics.hpp"
#include <math.h>

namespace rigid_body_motion {

    Eigen::MatrixXd VecToso3(const Eigen::VectorXd &omega){
        // Converts a 3-vector to its s03 representation
        assert(omega.size() == 3 && "Input vector must have 3 components");     // Check vector size 

        Eigen::MatrixXd so3mat(3, 3);
        so3mat << 0, -omega(2), omega(1),
              omega(2), 0, -omega(0),
              -omega(1), omega(0), 0;
        return so3mat;

    }

    Eigen::VectorXd so3ToVec(const Eigen::MatrixXd &so3mat){
        // Converts an so3 matrix to a vector
        assert(so3mat.rows() == 3 && so3mat.cols() == 3 && "Input matrix must be 3x3");
        assert((so3mat + so3mat.transpose()).isZero(1e-10) && "Input matrix must be skew symmetric");
        

        Eigen::VectorXd omega(3);
        omega << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
        return omega;

    }

    Eigen::MatrixXd Rodriguez(const Eigen::VectorXd &omega, const float &theta){
        // Implements Rodriguez formula to calculate a rotation matrix from an axis and angle
        assert(omega.size() == 3 && "Input vector must have 3 components");
        assert(fabs(omega_unit.norm() - 1.0) < 1e-10 && "Input vector must be a unit vector");

        Eigen::MatrixXd omega_hat = VecToso3(omega);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd R = I + sin(theta) * omega_hat + (1 - cos(theta)) * omega_hat * omega_hat;
        return R;
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