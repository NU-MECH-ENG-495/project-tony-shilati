#include "include/finger_model.hpp"
#include "include/open_chain_kinematics.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace fm {

    finger_model::finger_model()
        : finger_space_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          finger_body_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          tendon_routing_matrix(Eigen::MatrixXd::Zero(3, 6)),
          link_lengths(Eigen::VectorXd()),
          joint_angles(Eigen::VectorXd()),
          home_position_screw_axes_body(std::vector<Eigen::VectorXd>(3)),
          home_position_screw_axes_space(std::vector<Eigen::VectorXd>(3))
    {
        // Default constructor implementation
    }

    finger_model::finger_model(Eigen::VectorXd link_lengths, Eigen::VectorXd joint_angles)
        : finger_space_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          finger_body_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          tendon_routing_matrix(Eigen::MatrixXd::Zero(3, 6)),
          link_lengths(link_lengths),
          joint_angles(joint_angles),
          home_position_screw_axes_body(std::vector<Eigen::VectorXd>(3)),
          home_position_screw_axes_space(std::vector<Eigen::VectorXd>(3))
    {
        if (link_lengths.size() < 1 || link_lengths.size() > 4) {
            throw std::invalid_argument("link_lengths must have exactly 1-4 elements");
        }

        if (joint_angles.size() < 1 || joint_angles.size() > 4) {
            throw std::invalid_argument("joint_angles must have exactly 1-4 elements");
        }
    }
    
    finger_model::~finger_model()
    {
    }

    ////////////////////////////////////////////////////////////
    // Setter functions
    ////////////////////////////////////////////////////////////

    void finger_model::set_home_position_screw_axes_body(std::vector<Eigen::VectorXd> home_position_screw_axes) {
        if (home_position_screw_axes.size() < 1 || home_position_screw_axes.size() > 4) {
            throw std::invalid_argument("home_position_screw_axes must have exactly 1-4 elements");
        }
        this->home_position_screw_axes_body = home_position_screw_axes;
    }

    void finger_model::set_home_position_screw_axes_space(std::vector<Eigen::VectorXd> home_position_screw_axes) {
        if (home_position_screw_axes.size() < 1 || home_position_screw_axes.size() > 4) {
            throw std::invalid_argument("home_position_screw_axes must have exactly 1-4 elements");
        }
        this->home_position_screw_axes_space = home_position_screw_axes;
    }

    void finger_model::set_home_position_body_frame(Eigen::MatrixXd home_position_body_frame) {
        if (home_position_body_frame.rows() != 4 || home_position_body_frame.cols() != 4) {
            throw std::invalid_argument("home_position_body_frame must be a 4x4 matrix");
        }
        this->home_position_body_frame = home_position_body_frame;
    }

    void finger_model::set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix) {
        if (tendon_routing_matrix.rows() > 3 || tendon_routing_matrix.cols() > 6) {
            throw std::invalid_argument("tendon_routing_matrix allows no more than 3 rows and 6 columns");
        }
        this->tendon_routing_matrix = tendon_routing_matrix;
    }

    void finger_model::set_link_lengths(Eigen::VectorXd link_lengths) {
        if (link_lengths.size() < 1 || link_lengths.size() > 4) {
            throw std::invalid_argument("link_lengths must have exactly 1-4 elements");
        }
        this->link_lengths = link_lengths;
    }

    void finger_model::set_joint_angles(Eigen::VectorXd joint_angles) {
        if (joint_angles.size() < 1 || joint_angles.size() > 4) {
            throw std::invalid_argument("joint_angles must have exactly 1-4 elements");
        }
        this->joint_angles = joint_angles;
    }

    ////////////////////////////////////////////////////////////
    // Getter functions
    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd finger_model::get_finger_space_jacobian() {
        this->calculate_finger_space_jacobian();   // Update the finger space jacobian before returning it
        return finger_space_jacobian;
    }

    Eigen::MatrixXd finger_model::get_finger_body_jacobian() {
        this->calculate_finger_body_jacobian();    // Update the finger body jacobian before returning it
        return finger_body_jacobian;
    }

    Eigen::MatrixXd finger_model::get_tendon_routing_matrix() {
        return tendon_routing_matrix;
    }

    Eigen::VectorXd finger_model::get_link_lengths() {
        return link_lengths;
    }

    Eigen::VectorXd finger_model::get_joint_angles() {
        return joint_angles;
    }

    ////////////////////////////////////////////////////////////
    // Member functions
    ////////////////////////////////////////////////////////////

    // Update Space Jacobian
    void finger_model::calculate_finger_space_jacobian() {
        // Calculate Space Jacobian
    }

    // Update Body Jacobian
    void finger_model::calculate_finger_body_jacobian() {
        // Calculate Body Jacobian
    }

    Eigen::VectorXd finger_model::forward_kinematics_body() {
        // Calculate forward kinematics
        Eigen::MatrixXd T = open_chain_kinematics::FKin_Body(this->home_position_body_frame, this->home_position_screw_axes_body, this->joint_angles);
        return rigid_body_motion::Matrix_Logarithm(T);
    }

    Eigen::VectorXd finger_model::forward_kinematics_space() {
        // Calculate forward kinematics
        Eigen::MatrixXd T = open_chain_kinematics::FKin_Space(this->home_position_body_frame, this->home_position_screw_axes_space, this->joint_angles);
        return rigid_body_motion::Matrix_Logarithm(T);
    }

    Eigen::VectorXd finger_model::inverse_kinematics_body(const Eigen::MatrixXd& desired_pose) {
        return open_chain_kinematics::IKin_Body(this->home_position_body_frame, this->home_position_screw_axes_body, desired_pose);
    }

    Eigen::VectorXd finger_model::inverse_kinematics_space(const Eigen::MatrixXd& desired_pose) {
        return open_chain_kinematics::IKin_Space(this->home_position_body_frame, this->home_position_screw_axes_space, desired_pose);
    }

}