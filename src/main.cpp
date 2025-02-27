#include <iostream>
#include "finger_model.hpp"

int main() {
    std::cout << "Finger model program started." << std::endl;

    fm::finger_model finger;
    finger.initialize();
    finger.performAction();
    // You can add code here to create and use the finger_model class
    return 0;
}
