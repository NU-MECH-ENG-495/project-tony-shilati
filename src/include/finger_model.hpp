#ifndef FINGER_MODEL_HPP
#define FINGER_MODEL_HPP

#include <Eigen/Dense>
#include <vector>

namespace fm {

    class finger_model
    {
    public:
        finger_model(); // Default constructor
        finger_model(Eigen::MatrixXd home_position_body_frame, std::vector<double> link_lengths, std::vector<double> joint_angles); // Overloaded constructor
        ~finger_model();

        // Setters and Getters
        void set_home_position_screw_axes(std::vector<Eigen::VectorXd> home_position_screw_axes);
        void set_link_lengths(std::vector<double> link_lengths);
        void set_joint_angles(std::vector<double> joint_angles);
        Eigen::VectorXd get_link_lengths();
        Eigen::VectorXd get_joint_angles();

        void set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix);
        Eigen::MatrixXd get_finger_space_jacobian();
        Eigen::MatrixXd get_finger_body_jacobian();
        Eigen::MatrixXd get_tendon_routing_matrix();

        // Member functions
        void calculate_finger_space_jacobian();
        void calculate_finger_body_jacobian();
        Eigen::VectorXd forward_kinematics_body(Eigen::MatrixXd M, std::vector<Eigen::VectorXd> Blist, Eigen::VectorXd thetalist);
        Eigen::VectorXd forward_kinematics_space(Eigen::MatrixXd M, std::vector<Eigen::VectorXd> Slist, Eigen::VectorXd thetalist);


    private:
        // Finger geometric information
        Eigen::MatrixXd home_position_body_frame;
        std::vector<Eigen::VectorXd> home_position_screw_axes_body;
        std::vector<Eigen::VectorXd> home_position_screw_axes_space;
        Eigen::VectorXd link_lengths; 
        Eigen::VectorXd joint_angles; 

        // Finger mappings
        Eigen::MatrixXd finger_space_jacobian;
        Eigen::MatrixXd finger_body_jacobian;
        Eigen::MatrixXd tendon_routing_matrix;
        Eigen::MatrixXd motor_shaft_matrix;

        
    };

}

#endif // FINGER_MODEL_HPP