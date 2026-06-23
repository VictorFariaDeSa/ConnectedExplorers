#ifndef LAPLACIAN_MATRIX_HANDLER_HPP
#define LAPLACIAN_MATRIX_HANDLER_HPP

#include <Eigen/Dense>
#include <mutex>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace connected_explorers_utils{

class LaplacianMatrixHandler
{
private:
    // Constructor parameters ---
    int number_of_robots_;
    int n_dims_;

    // Laplacian matrixes ---
    std::mutex matrix_mutex_;
    Eigen::MatrixXd laplacian_matrix_;
    Eigen::MatrixXd degree_matrix_;
    Eigen::MatrixXd adjacency_matrix_;

    // Gradient calculation ---
    std::mutex derivatives_mutex_;
    std::vector<Eigen::MatrixXd> connections_gradient_vector_;
    std::vector<double> fiedler_gradient_;

public:
    LaplacianMatrixHandler(int n_robots,int n_dims);
    ~LaplacianMatrixHandler();

    // Init functions ---
    void InitMatrixes();
    void InitGradientData();
    void InitGradientVector();



    void UpdateGradientData(
        int robot_index, 
        int conn_index,
        const Eigen::RowVectorXd& newRow
    );
    void UpdateGradientVector();



    void UpdateConnWeight(float new_weight, int i_1, int i_2);
    void UpdateInternalMath();
    void UpdateLaplacianMatrix();
    void UpdateFiedlerVectorValue(int index, double new_value);


    // Mudar ambos esses nomes
    void SetLaplacianFromMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    std_msgs::msg::Float64MultiArray GetLaplacianMsg();
    
    
    // Getters & Setters---
    double GetFiedlerValue();
    Eigen::VectorXd GetFiedlerVector();
    std::vector<double> GetGradient();
    Eigen::MatrixXd GetLaplacianMatrix();
    Eigen::MatrixXd GetAdjacencyMatrix();

    
    
};  
}



#endif //LAPLACIAN_MATRIX_HANDLER_HPP
