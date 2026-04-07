#include "connected_explorers_utils/LaplacianMatrixHandler.hpp"


namespace connected_explorers_utils{

LaplacianMatrixHandler::LaplacianMatrixHandler(int n_robots):
number_of_robots_(n_robots)
{
    InitMatrixes();
}

LaplacianMatrixHandler::~LaplacianMatrixHandler()
{
}

void LaplacianMatrixHandler::InitMatrixes(){
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    laplacian_matrix_.resize(number_of_robots_, number_of_robots_);
    laplacian_matrix_.setZero();

    degree_matrix_.resize(number_of_robots_, number_of_robots_);
    degree_matrix_.setZero();

    adjacency_matrix_.resize(number_of_robots_, number_of_robots_);
    adjacency_matrix_.setZero();
}

void LaplacianMatrixHandler::UpdateInternalMath() {
    // Update Degree Matrix
    degree_matrix_.setZero();
    for (int i = 0; i < number_of_robots_; ++i) {
        degree_matrix_(i, i) = adjacency_matrix_.row(i).sum();
    }
    laplacian_matrix_ = degree_matrix_ - adjacency_matrix_;
}

void LaplacianMatrixHandler::UpdateLaplacianMatrix(){
    laplacian_matrix_ = degree_matrix_ - adjacency_matrix_;
}

void LaplacianMatrixHandler::UpdateConnWeight(float new_weight, int i_1, int i_2) {
    if (i_1 >= number_of_robots_ || i_2 >= number_of_robots_) return;

    {
        std::lock_guard<std::mutex> lock(matrix_mutex_);
        
        // Update the edge
        adjacency_matrix_(i_1, i_2) = new_weight;
        adjacency_matrix_(i_2, i_1) = new_weight;

        // Perform all dependent math while the lock is still held
        UpdateInternalMath();
    }
}

double LaplacianMatrixHandler::GetFiedlerValue() {
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(laplacian_matrix_);
    
    if (solver.info() != Eigen::Success) return -1.0;

    return solver.eigenvalues()(1);
}

Eigen::VectorXd LaplacianMatrixHandler::GetFiedlerVector() {
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(laplacian_matrix_);
    
    if (solver.info() != Eigen::Success) return Eigen::VectorXd();

    return solver.eigenvectors().col(1);
}

std_msgs::msg::Float64MultiArray LaplacianMatrixHandler::GetLaplacianMsg() {
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    
    std_msgs::msg::Float64MultiArray msg;

    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = laplacian_matrix_.rows();
    msg.layout.dim[0].stride = laplacian_matrix_.rows() * laplacian_matrix_.cols();

    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = laplacian_matrix_.cols();
    msg.layout.dim[1].stride = laplacian_matrix_.cols();

    msg.data.reserve(laplacian_matrix_.size());
    for (int i = 0; i < laplacian_matrix_.rows(); ++i) {
        for (int j = 0; j < laplacian_matrix_.cols(); ++j) {
            msg.data.push_back(laplacian_matrix_(i, j));
        }
    }

    return msg;
}

Eigen::MatrixXd LaplacianMatrixHandler::GetLaplacianMatrix() {
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    return laplacian_matrix_;
}

Eigen::MatrixXd LaplacianMatrixHandler::GetAdjacencyMatrix() {
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    return adjacency_matrix_;
}
}