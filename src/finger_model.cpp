#include <iostream>
#include <eigen.h>
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

        void set_finger_space_jacobian(Eigen::MatrixXd finger__space_jacobian);
        void set_finger_body_jacobian(Eigen::MatrixXd finger_body_jacobian);
        void set_tendon_routing_matrix(Eigen::MatrixXd tendon_routing_matrix);
        void set_link_lengths(std::vector<double> link_lengths);
        Eigen::MatrixXd get_finger_space_jacobian();
        Eigen::MatrixXd get_finger_body_jacobian();
        Eigen::MatrixXd get_tendon_routing_matrix();
        void get_link_lengths();

    private:
        Eigen::MatrixXd finger_space_jacobian;
        Eigen::MatrixXd finger_body_jacobian;
        Eigen::MatrixXd tendon_routing_matrix;
        std::vector<double> link_lengths;
    };

    finger_model::finger_model(/* args */)
    {
    }
    
    finger_model::~finger_model()
    {
    }

    void finger_model::set_finger_space_jacobian(Eigen::MatrixXd finger_space_jacobian) {
        if (finger_space_jacobian.rows() != 6) {
            throw std::invalid_argument("finger_space_jacobian must be a 6xN matrix");
        }
        this->finger_space_jacobian = finger_space_jacobian;
    }

    void finger_model::set_finger_body_jacobian(Eigen::MatrixXd finger_body_jacobian) {
        if (finger_body_jacobian.rows() != 6) {
            throw std::invalid_argument("finger_body_jacobian must be a 6xN matrix");
        }
        this->finger_body_jacobian = finger_body_jacobian;
    }

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

    Eigen::MatrixXd finger_model::get_finger_space_jacobian() {
        return finger_space_jacobian;
    }

    Eigen::MatrixXd finger_model::get_finger_body_jacobian() {
        return finger_body_jacobian;
    }

    Eigen::MatrixXd finger_model::get_tendon_routing_matrix() {
        return tendon_routing_matrix;
    }

    std::vector<double> finger_model::get_link_lengths() {
        return link_lengths;
    }

}