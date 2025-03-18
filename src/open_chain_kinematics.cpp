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
        Eigen::VectorXd omega_hat = omega.normalized();

        Eigen::MatrixXd omega_so3 = VecToso3(omega_hat);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd R = I + sin(theta) * omega_so3 + (1 - cos(theta)) * omega_so3 * omega_so3;
        return R;
    }

    Eigen::VectorXd RotationLogarithm (const Eigen::MatrixXd &R){
        // Calculates the logarithm of a rotation matrix
        assert(R.rows() == 3 && R.cols() == 3 && "Input matrix must be 3x3");
        assert(R.determinant() == 1 && "Input matrix must be a rotation matrix");

        // If theta is 0, return a zero vector
        if (R.isApprox(Eigen::MatrixXd::Identity(3, 3), 1e-10)) {
            return Eigen::VectorXd::Zero(3);
        }

        // If theta is not zero: logarithm
        double theta = acos((R.trace() - 1) / 2);
        Eigen::MatrixXd omega_so3 = (R - R.transpose()) / (2 * sin(theta));
        Eigen::VectorXd omega = so3ToVec(omega_so3);
        return omega * theta;
    }


    Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &T){
        // Calculates the adjoint representation of a transformation matrix
        assert(T.rows() == 4 && T.cols() == 4 && "Input matrix must be 4x4");

        Eigen::MatrixXd R = T.block(0, 0, 2, 2);
        Eigen::MatrixXd p = T.block(0, 3, 2, 3);
        Eigen::MatrixXd p_so3 = VecToso3(p);
        Eigen::MatrixXd Ad_T(6, 6);
        Ad_T << R, Eigen::MatrixXd::Zero(3, 3),
            p_so3 * R, R;
        return Ad_T;

    }

    Eigen::MatrixXd Matrix_Exponential(const Eigen::VectorXd &S, const float &theta){
        // Calculates the matrix exponential of a rigid body motion
        assert(S.size() == 6 && "Input vector S must have 6 components");

        Eigen::VectorXd omega = S.head(3);
        Eigen::VectorXd v = S.tail(3);

        Eigen::MatrixXd omega_so3 = VecToso3(omega);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd R = Rodriguez(omega_so3, theta);
        Eigen::MatrixXd V = (I * theta + (1 - cos(theta)) * omega_so3 + (theta - sin(theta)) * omega_so3 * omega_so3) * v;

        Eigen::MatrixXd exp_S_theta(4, 4);
        exp_S_theta << R, V,
                   0, 0, 0, 1;

        return exp_S_theta;

    }

    Eigen::MatrixXd Matrix_Logarithm(const Eigen::MatrixXd &T){
        // Calculates the matrix logarithm of a rigid body motion

        // Decompose the input matrix into rotation and translation
        Eigen::MatrixXd R = T.block(0, 0, 3, 3);
        Eigen::VectorXd p = T.block(0, 3, 2, 3);

        // Verify that the input matrix is in SE3
        assert(T.rows() == 4 && T.cols() == 4 && "Input matrix must be in SE3");
        assert(T.block(3, 0, 3, 3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-10) && "Input matrix must be in SE3");
        assert(R.transpose() * R == Eigen::MatrixXd::Identity(3, 3) && "Input matrix must be in SE3");


        // Calculate the matrix logarithm
        double theta = acos((R.trace() - 1) / 2);

    
        Eigen::VectorXd omega = RotationLogarithm(R);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd G_inv = (1/theta) * I - 1/2 * VecToso3(omega) + (1/theta - 1/2 / tan(theta/2)) * VecToso3(omega) * VecToso3(omega);
        Eigen::VectorXd v = G_inv * p;
        Eigen::VectorXd S(6);
        S << omega, v;
        return S * theta;

        

    }

}

namespace open_chain_kinematics {

    Eigen::MatrixXd FKin_Space(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> S_list, const Eigen::VectorXd theta_list) {
        // Forward kinematics in the space frame
        assert(S_list.size() == theta_list.size() && "S_list and theta_list must have the same size");
        for (const auto& S : S_list) {
            assert(S.size() == 6 && "Each element of S_list must be a 6-vector");
        }

        // Product of exponentials formula
        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
        for (int i = 0; i < S_list.size(); i++) {
            T = T * rigid_body_motion::Matrix_Exponential(S_list[i], theta_list[i]);
        }

        T = T * M;

        // Return the Adjoint of the final transformation matrix
        return rigid_body_motion::Adjoint(T);


    }

    Eigen::MatrixXd FKin_Body(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> B_list, const Eigen::VectorXd theta_list) {
        // Forward kinematics in the body frame
        assert(B_list.size() == theta_list.size() && "B_list and theta_list must have the same size");
        for (const auto& B : B_list) {
            assert(B.size() == 6 && "Each element of B_list must be a 6-vector");
        }

        // Product of exponentials formula
        Eigen::MatrixXd T = M;
        for (int i = 0; i < B_list.size(); i++) {
            T = T * rigid_body_motion::Matrix_Exponential(B_list[i], theta_list[i]);
        }

        T = M * T;

        // Return the Adjoint of the final transformation matrix
        return rigid_body_motion::Adjoint(T);
 
    }

} // namespace open_chain_kinematics