#include <iostream>
#include "include/finger_model.hpp"
#include "include/render_finger.hpp"
#include <Eigen/Dense>
#include <math.h>
#include <vector>

int main(int argc, char *argv[]) {
    std::cout << "Finger model program started." << std::endl;

    // Create a vector of link lengths
    Eigen::VectorXd link_lengths(3); link_lengths << 0.046, 0.032, 0.025;

    // Create a vector of joint angles
    Eigen::VectorXd joint_angles(3); joint_angles << 0.0, 0.0, 0.0;

    // Create a finger model
    fm::finger_model finger(link_lengths, joint_angles);

    // Create a vector containing the fingers's home position screw axes
    Eigen::VectorXd S1(6); S1 << 0, 0, 1, 0, 0, 0;
    Eigen::VectorXd S2(6); S2 << 0, 0, 1, 0, -link_lengths(0), 0;
    Eigen::VectorXd S3(6); S3 << 0, 0, 1, 0, -link_lengths(0) - link_lengths(1), 0;
    std::vector<Eigen::VectorXd> home_position_screw_axes;
    home_position_screw_axes.push_back(S1);
    home_position_screw_axes.push_back(S2);
    home_position_screw_axes.push_back(S3);

    // Create a 4x4 SE3 matrix representing the finger's home position in the body frame
    Eigen::MatrixXd home_position_body_frame(4, 4);
    home_position_body_frame << 1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;

    // Set the finger's home position screw axes and home position in the body frame
    finger.set_home_position_screw_axes_space(home_position_screw_axes);
    finger.set_home_position_body_frame(home_position_body_frame);

    std::cout << "Link lengths: ";
    for (const auto& length : finger.get_link_lengths()) {
        std::cout << length << " ";
    }
    std::cout << std::endl;

    /*
    // Define desired end effector pose
    Eigen::MatrixXd desired_pose = Eigen::MatrixXd::Identity(4, 4);
    desired_pose(0, 3) = 0.103; // Set desired x position
    desired_pose(1, 3) = 0.0; // Set desired y position
    desired_pose(2, 3) = 0.0; // Set desired z position

    // Calculate inverse kinematics
    Eigen::VectorXd ik_result = finger.inverse_kinematics_body(desired_pose);
    

    // Print the result
    std::cout << "Inverse Kinematics Result (Body Frame): " << ik_result.transpose() << std::endl;
    */
  
    // Render the finger
    render_finger();

    return 0;
}
