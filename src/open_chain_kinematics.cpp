#include <iostream>
#include <Eigen/Dense>
#include "include/open_chain_kinematics.hpp"
#include <math.h>

namespace rigid_body_motion {

    /**
     * @brief Converts a 3-vector to its so3 representation.
     * @param omega A 3-vector.
     * @return A 3x3 skew-symmetric matrix.
     */
    Eigen::MatrixXd VecToso3(const Eigen::VectorXd &omega){
        // Converts a 3-vector to its s03 representation
        assert(omega.size() == 3 && "Input vector must have 3 components");     // Check vector size 

        Eigen::MatrixXd so3mat(3, 3);
        so3mat << 0, -omega(2), omega(1),
              omega(2), 0, -omega(0),
              -omega(1), omega(0), 0;
        return so3mat;

    }

    /**
     * @brief Converts an so3 matrix to a vector.
     * @param so3mat A 3x3 skew-symmetric matrix.
     * @return A 3-vector.
     */
    Eigen::VectorXd so3ToVec(const Eigen::MatrixXd &so3mat){
        // Converts an so3 matrix to a vector
        assert(so3mat.rows() == 3 && so3mat.cols() == 3 && "Input matrix must be 3x3");
        assert((so3mat + so3mat.transpose()).isZero(1e-10) && "Input matrix must be skew symmetric");
        

        Eigen::VectorXd omega(3);
        omega << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
        return omega;

    }

    /**
     * @brief Implements Rodriguez formula to calculate a rotation matrix from an axis and angle.
     * @param omega A 3-vector representing the axis of rotation.
     * @param theta The angle of rotation in radians.
     * @return A 3x3 rotation matrix.
     */
    Eigen::MatrixXd Rodriguez(const Eigen::VectorXd &omega, const float &theta){
        // Implements Rodriguez formula to calculate a rotation matrix from an axis and angle
        assert(omega.size() == 3 && "Input vector must have 3 components");

        Eigen::MatrixXd omega_so3 = VecToso3(omega);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd R = I + sin(theta) * omega_so3 + (1 - cos(theta)) * omega_so3 * omega_so3;
        return R;
    }

    /**
     * @brief Calculates the logarithm of a rotation matrix.
     * @param R A 3x3 rotation matrix.
     * @return A 3-vector representing the rotation axis multiplied by the rotation angle.
     */
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

    /**
     * @brief Calculates the adjoint representation of a transformation matrix.
     * @param T A 4x4 transformation matrix.
     * @return A 6x6 adjoint matrix.
     */
    Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &T){
        // Calculates the adjoint representation of a transformation matrix
        assert(T.rows() == 4 && T.cols() == 4 && "Input matrix must be 4x4");

        Eigen::MatrixXd R = T.block(0, 0, 3, 3);
        Eigen::VectorXd p = T.block<3, 1>(0, 3);
        Eigen::MatrixXd p_so3 = VecToso3(p);
        Eigen::MatrixXd Ad_T(6, 6);
        Ad_T << R, Eigen::MatrixXd::Zero(3, 3),
            p_so3 * R, R;
        return Ad_T;

    }

    /**
     * @brief Calculates the matrix exponential of a rigid body motion.
     * @param S A 6-vector representing the twist.
     * @param theta The angle of rotation in radians.
     * @return A 4x4 transformation matrix.
     */
    Eigen::MatrixXd Matrix_Exponential(const Eigen::VectorXd &S, const float &theta){
        // Calculates the matrix exponential of a rigid body motion
        assert(S.size() == 6 && "Input vector S must have 6 components");

        Eigen::VectorXd omega = S.head(3);
        Eigen::VectorXd v = S.tail(3);

        Eigen::MatrixXd omega_so3 = VecToso3(omega);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd R = Rodriguez(omega, theta);
        Eigen::VectorXd V = (I * theta + (1 - cos(theta)) * omega_so3 + (theta - sin(theta)) * omega_so3 * omega_so3) * v;

        Eigen::MatrixXd exp_S_theta(4, 4);
        exp_S_theta << R, V,
                   0, 0, 0, 1;

        return exp_S_theta;

    }

    /**
     * @brief Calculates the matrix logarithm of a rigid body motion.
     * @param T A 4x4 transformation matrix.
     * @return A 6-vector representing the twist multiplied by the rotation angle.
     */
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

    /**
     * @brief Forward kinematics in the space frame.
     * @param M The home configuration (position and orientation) of the end-effector.
     * @param S_list A list of screw axes in the space frame.
     * @param theta_list A list of joint angles/positions.
     * @return The transformation matrix representing the end-effector frame relative to the space frame.
     */
    Eigen::MatrixXd FKin_Space(const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> S_list, const Eigen::VectorXd theta_list) {
        // Forward kinematics in the space frame
        assert(M.rows() == 4 && M.cols() == 4 && "Input matrix M must be 4x4");
        assert(M.block(0, 0, 3, 3).transpose() * M.block(0, 0, 3, 3) == Eigen::MatrixXd::Identity(3, 3) && "Rotation part of M must be orthogonal");
        assert(M.block(3, 0, 1, 3).isZero(1e-10) && M(3, 3) == 1 && "Last row of M must be [0, 0, 0, 1]");
        assert(S_list.size() == theta_list.size() && "S_list and theta_list must have the same size");
        for (const auto& S : S_list) {
            assert(S.size() == 6 && "Each element of S_list must be a 6-vector");
        }

        // Product of exponentials formula
        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
        for (int i = 0; i < S_list.size(); i++) {
            T = T * rigid_body_motion::Matrix_Exponential(S_list[i], theta_list(i));
        }

        T = T * M;

        // Return the final transformation matrix
        return T;


    }

    /**
     * @brief Forward kinematics in the body frame.
     * @param M The home configuration (position and orientation) of the end-effector.
     * @param B_list A list of screw axes in the body frame.
     * @param theta_list A list of joint angles/positions.
     * @return The transformation matrix representing the end-effector frame relative to the body frame.
     */
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

        // Return the final transformation matrix
        return T;
 
    }

    /**
     * @brief Inverse kinematics in the space frame.
     * @param M The home configuration (position and orientation) of the end-effector.
     * @param S_list A list of screw axes in the space frame.
     * @param T The desired end-effector configuration.
     * @return A list of joint angles/positions that achieve the desired end-effector configuration.
     */
    Eigen::VectorXd IKin_Space (const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> S_list, const Eigen::MatrixXd T) {
        // Inverse kinematics in the space frame
        assert(S_list.size() == T.cols() && "S_list and T must have the same number of columns");
        for (const auto& S : S_list) {
            assert(S.size() == 6 && "Each element of S_list must be a 6-vector");
        }

        // Initialize the theta list
        Eigen::VectorXd theta_list(T.cols());
        theta_list.setZero();

        // Initialize the error
        double error = 1e10;
        double tol = 1e-6;
        int max_iter = 1000;
        int iter = 0;

        // Iterate until the error is below the tolerance
        while (error > tol && iter < max_iter) {
            // Calculate the current transformation matrix
            Eigen::MatrixXd T_current = FKin_Space(M, S_list, theta_list);

            // Calculate the error
            Eigen::MatrixXd T_inv = T_current.inverse();
            Eigen::MatrixXd T_error = T_inv * T;

            // Calculate the twist
            Eigen::VectorXd S_error = rigid_body_motion::Matrix_Logarithm(T_error);

            // Update the theta list
            theta_list = theta_list + S_error;

            // Update the error
            error = S_error.norm();
            iter++;
        }

        return theta_list;

    }

    /**
     * @brief Inverse kinematics in the body frame.
     * @param M The home configuration (position and orientation) of the end-effector.
     * @param B_list A list of screw axes in the body frame.
     * @param T The desired end-effector configuration.
     * @return A list of joint angles/positions that achieve the desired end-effector configuration.
     */
    Eigen::VectorXd IKin_Body (const Eigen::MatrixXd M, const std::vector<Eigen::VectorXd> B_list, const Eigen::MatrixXd T) {
        // Inverse kinematics in the body frame
        /*
        assert(B_list.size() == T.cols() && "B_list and T must have the same number of columns");
        for (const auto& B : B_list) {
            assert(B.size() == 6 && "Each element of B_list must be a 6-vector");
        }
            */

        // Initialize the theta list
        Eigen::VectorXd theta_list(T.cols());
        theta_list.setZero();

        // Initialize the error
        double error = 1e10;
        double tol = 1e-6;
        int max_iter = 1000;
        int iter = 0;

        // Iterate until the error is below the tolerance
        while (error > tol && iter < max_iter) {
            // Calculate the current transformation matrix
            Eigen::MatrixXd T_current = FKin_Body(M, B_list, theta_list);

            // Calculate the error
            Eigen::MatrixXd T_inv = T_current.inverse();
            Eigen::MatrixXd T_error = T_inv * T;

            // Calculate the twist
            Eigen::VectorXd B_error = rigid_body_motion::Matrix_Logarithm(T_error);

            // Update the theta list
            theta_list = theta_list + B_error;

            // Update the error
            error = B_error.norm();
            iter++;
        }

        return theta_list;

    }

} // namespace open_chain_kinematics