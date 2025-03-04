#ifndef FINGER_MODEL_HPP
#define FINGER_MODEL_HPP

#include <Eigen/Dense>
#include <vector>

namespace fm {

    class finger_model
    {
    public:
        finger_model(Eigen::MatrixXd home_position_body_frame, std::vector<Eigen::VectorXd> home_position_screw_axes, std::vector<double> link_lengths, std::vector<double> joint_angles);
        ~finger_model();

        // Setters and Getters
        void set_home_position_screw_axes(std::vector<Eigen::VectorXd> home_position_screw_axes);
        void set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix);
        void set_link_lengths(std::vector<double> link_lengths);
        void set_joint_angles(std::vector<double> joint_angles);
        Eigen::MatrixXd get_finger_space_jacobian();
        Eigen::MatrixXd get_finger_body_jacobian();
        Eigen::MatrixXd get_tendon_routing_matrix();
        std::vector<double> get_link_lengths();
        std::vector<double> get_joint_angles();

        // Member functions
        void calculate_finger_space_jacobian();
        void calculate_finger_body_jacobian();
        Eigen::VectorXd forward_kinematics_body(Eigen::MatrixXd M, std::vector<Eigen::VectorXd> Blist, std::vector<double> thetalist);
        Eigen::VectorXd forward_kinematics_space(Eigen::MatrixXd M, std::vector<Eigen::VectorXd> Slist, std::vector<double> thetalist);


    private:
        Eigen::MatrixXd home_position_body_frame;
        std::vector<Eigen::VectorXd> home_position_screw_axes; // Home position screw axes from proximal to distal
        Eigen::MatrixXd finger_space_jacobian;
        Eigen::MatrixXd finger_body_jacobian;
        Eigen::MatrixXd tendon_routing_matrix;
        std::vector<double> link_lengths; // Finger link lengths from proximal to distal
        std::vector<double> joint_angles; // Joint angles in radians from proximal to distal
    };

}

#endif // FINGER_MODEL_HPP