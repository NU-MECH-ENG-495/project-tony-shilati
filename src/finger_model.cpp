#include "include/finger_model.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace fm {

    finger_model::finger_model(Eigen::MatrixXd home_position_body_frame, std::vector<Eigen::VectorXd> home_position_screw_axes, std::vector<double> link_lengths, std::vector<double> joint_angles)
        : finger_space_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          finger_body_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          tendon_routing_matrix(Eigen::MatrixXd::Zero(3, 6)),
          link_lengths(4, 0.0),
          joint_angles(3, 0.0) // Initialize joint_angles with 3 elements
    {
        if (home_position_body_frame.rows() != 4 || home_position_body_frame.cols() != 4) {
            throw std::invalid_argument("home_position_body_frame must be a 4x4 SE3 matrix");
        }
        if (home_position_screw_axes.size() > 4 || home_position_screw_axes.size() < 1) {
            throw std::invalid_argument("home_position_screw_axes must have exactly 1 - 4 elements");
        }
        for (const auto& vector : home_position_screw_axes) {
            if (vector.size() != 6) {
                throw std::invalid_argument("Each vector in home_position_screw_axes must have exactly 6 elements");
            }
        }
        if (link_lengths.size() < 1 || link_lengths.size() > 4) {
            throw std::invalid_argument("link_lengths must have exactly 1-4 elements");
        }
        if (joint_angles.size() < 1 || joint_angles.size() > 4) {
            throw std::invalid_argument("joint_angles must have exactly 1-4 elements");
        }
        this->home_position_body_frame = home_position_body_frame; // Initialize home_position_body_frame with the passed matrix
        this->home_position_screw_axes = home_position_screw_axes; // Initialize home_position_screw_axes with the passed vector
        this->link_lengths = link_lengths; // Initialize link_lengths with the passed vector
        this->joint_angles = joint_angles; // Initialize joint_angles with the passed vector
    }
    
    finger_model::~finger_model()
    {
    }

    ////////////////////////////////////////////////////////////
    // Setter functions
    ////////////////////////////////////////////////////////////

    void finger_model::set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix) {
        if (tendon_routing_matrix.rows() > 3 || tendon_routing_matrix.cols() > 6) {
            throw std::invalid_argument("tendon_routing_matrix allows no more than 3 rows and 6 columns");
        }
        this->tendon_routing_matrix = tendon_routing_matrix;
    }

    void finger_model::set_link_lengths(std::vector<double> link_lengths) {
        if (link_lengths.size() < 1 || link_lengths.size() > 4) {
            throw std::invalid_argument("link_lengths must have exactly 1-4 elements");
        }
        this->link_lengths = link_lengths;
    }

    void finger_model::set_joint_angles(std::vector<double> joint_angles) {
        if (joint_angles.size() != 3) {
            throw std::invalid_argument("joint_angles must have exactly 3 elements");
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

    std::vector<double> finger_model::get_link_lengths() {
        return link_lengths;
    }

    std::vector<double> finger_model::get_joint_angles() {
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

}