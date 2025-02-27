#include "finger_model.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <vector>

/*
    This class is used to model the finger. It contains the finger jacobian and the tendon routing matrix.
    The finger jacobian is a matrix that relates the joint velocities to the end effector velocity.
    The tendon routing matrix is a matrix that relates the tendon forces to the joint torques.
    The link lengths are the lengths of the links in the finger. Standard SI units are used.
*/

namespace fm {
    
    class finger_model
    {
    public:
        finger_model(/* args */);
        ~finger_model();

        // Setters and Getters
        void set_finger_space_jacobian(Eigen::MatrixXd finger__space_jacobian);
        void set_finger_body_jacobian(Eigen::MatrixXd finger_body_jacobian);
        void set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix);
        void set_link_lengths(std::vector<double> link_lengths);
        void set_joint_angles(std::vector<double> joint_angles);
        Eigen::MatrixXd get_finger_space_jacobian();
        Eigen::MatrixXd get_finger_body_jacobian();
        Eigen::MatrixXd get_tendon_routing_matrix();
        std::vector<double> get_link_lengths();
        std::vector<double> get_joint_angles();

        // Member functions
        void update_finger_space_jacobian();
        void update_finger_body_jacobian();

    private:
        Eigen::MatrixXd finger_space_jacobian;
        Eigen::MatrixXd finger_body_jacobian;
        Eigen::MatrixXd tendon_routing_matrix;
        std::vector<double> link_lengths; // Finger link lengths from proximal to distal
        std::vector<double> joint_angles; // Joint angles in radians from proximal to distal
    };

    finger_model::finger_model()
        : finger_space_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          finger_body_jacobian(Eigen::MatrixXd::Zero(6, 3)),
          tendon_routing_matrix(Eigen::MatrixXd::Zero(3, 4)),
          link_lengths(4, 0.0)
    {
    }
    
    finger_model::~finger_model()
    {
    }

    ////////////////////////////////////////////////////////////
    // Getter functions
    ////////////////////////////////////////////////////////////

    void finger_model::set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix) {
        if (tendon_routing_matrix.rows() > 3 || tendon_routing_matrix.cols() > 6) {
            throw std::invalid_argument("tendon_routing_matrix allows no more than 3 rows and 6 columns");
        }
        this->tendon_routing_matrix = tendon_routing_matrix;
    }

    void finger_model::set_link_lengths(std::vector<double> link_lengths) {
        if (link_lengths.size() != 4) {
            throw std::invalid_argument("link_lengths must have exactly 4 elements");
        }
        this->link_lengths = link_lengths;
    }

    void finger_model::set_joint_angles(std::vector<double> joint_angles) {
        if (joint_angles.size() != 3) {
            throw std::invalid_argument("joint_angles must have exactly 4 elements");
        }
        this->joint_angles = joint_angles;
    }

    ////////////////////////////////////////////////////////////
    // Getter functions
    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd finger_model::get_finger_space_jacobian() {
        this->update_finger_space_jacobian();   // Update the finger space jacobian before returning it
        return finger_space_jacobian;
    }

    Eigen::MatrixXd finger_model::get_finger_body_jacobian() {
        this->update_finger_body_jacobian();    // Update the finger body jacobian before returning it
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
    // Memeber functions
    ////////////////////////////////////////////////////////////

    // Update Space Jacobian
    void finger_model::update_finger_space_jacobian() {
        // Update Space Jacobian
    }

    // Update Body Jacobian
    void finger_model::update_finger_body_jacobian() {
        // Update Body Jacobian
    }

}