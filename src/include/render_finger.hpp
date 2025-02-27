#ifndef FINGER_MODEL_H
#define FINGER_MODEL_H

#include <Eigen/Dense>
#include <vector>

namespace fm {
    
    class finger_model
    {
    public:
        finger_model(/* args */);
        ~finger_model();

        void set_finger_space_jacobian(Eigen::MatrixXd finger__space_jacobian);
        void set_finger_body_jacobian(Eigen::MatrixXd finger_body_jacobian);
        void set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix);
        void set_link_lengths(std::vector<double> link_lengths);
        Eigen::MatrixXd get_finger_space_jacobian();
        Eigen::MatrixXd get_finger_body_jacobian();
        Eigen::MatrixXd get_tendon_routing_matrix();
        std::vector<double> get_link_lengths();

    private:
        Eigen::MatrixXd finger_space_jacobian;
        Eigen::MatrixXd finger_body_jacobian;
        Eigen::MatrixXd tendon_routing_matrix;
        std::vector<double> link_lengths;
    };

}

#endif // FINGER_MODEL_H
