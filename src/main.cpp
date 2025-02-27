#include <iostream>
#include "include/finger_model.hpp"
#include <Eigen/Dense>

int main() {
    std::cout << "Finger model program started." << std::endl;

    // Create a finger model
    fm::finger_model finger;
    finger.set_link_lengths({1.0, 1.0, 1.0, 1.0});
    finger.set_joint_angles({0.0, 0.0, 0.0});

    std::cout << "Link lengths: ";
    for (const auto& length : finger.get_link_lengths()) {
        std::cout << length << " ";
    }
    std::cout << std::endl;
    
    // You can add code here to create and use the finger_model class
    return 0;
}
