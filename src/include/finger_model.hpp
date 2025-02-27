#ifndef FINGER_MODEL_HPP
#define FINGER_MODEL_HPP

#include <Eigen/Dense>
#include <vector>

namespace fm {

    class finger_model
    {
    public:
        finger_model();
        ~finger_model();

        // Setters and Getters
        void set_finger_space_jacobian(Eigen::MatrixXd finger_space_jacobian);
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

}

#endif // FINGER_MODEL_HPP