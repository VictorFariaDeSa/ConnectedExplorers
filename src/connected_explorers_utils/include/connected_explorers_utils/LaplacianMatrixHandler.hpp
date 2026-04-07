#ifndef LAPLACIAN_MATRIX_HANDLER_HPP
#define LAPLACIAN_MATRIX_HANDLER_HPP

#include <Eigen/Dense>
#include <mutex>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace connected_explorers_utils{

class LaplacianMatrixHandler
{
private:
    int number_of_robots_;

    Eigen::MatrixXd laplacian_matrix_;
    Eigen::MatrixXd degree_matrix_;
    Eigen::MatrixXd adjacency_matrix_;

    std::mutex matrix_mutex_;


public:
    LaplacianMatrixHandler(int n_robots);
    ~LaplacianMatrixHandler();



    void InitMatrixes();
    void UpdateConnWeight(float new_weight, int i_1, int i_2);
    void UpdateInternalMath();
    void UpdateLaplacianMatrix();
    double GetFiedlerValue();
    Eigen::VectorXd GetFiedlerVector();
    std_msgs::msg::Float64MultiArray GetLaplacianMsg();
    Eigen::MatrixXd GetLaplacianMatrix();
    Eigen::MatrixXd GetAdjacencyMatrix();
};  
}



#endif //LAPLACIAN_MATRIX_HANDLER_HPP
