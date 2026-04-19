#include "connected_explorers_laplacian_matrix/LaplacianMatrixHandler.hpp"


static double GetFiedler(Eigen::MatrixXd matrix){
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(matrix);
    if (solver.info() != Eigen::Success) return -1.0;
    return solver.eigenvalues()(1);
}

namespace connected_explorers_utils{

LaplacianMatrixHandler::LaplacianMatrixHandler(int n_robots,int n_dims):
number_of_robots_(n_robots),n_dims_(n_dims)
{
    InitMatrixes();
    InitGradientData();
    InitGradientVector();
}

LaplacianMatrixHandler::~LaplacianMatrixHandler()
{
}


/*******************************************************************************
* Init functions
*******************************************************************************/
void LaplacianMatrixHandler::InitMatrixes(){
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    laplacian_matrix_.resize(number_of_robots_, number_of_robots_);
    laplacian_matrix_.setZero();

    degree_matrix_.resize(number_of_robots_, number_of_robots_);
    degree_matrix_.setZero();

    adjacency_matrix_.resize(number_of_robots_, number_of_robots_);
    adjacency_matrix_.setZero();
}

void LaplacianMatrixHandler::InitGradientData(){
    std::lock_guard<std::mutex> lock(derivatives_mutex_);
    connections_gradient_vector_.resize(number_of_robots_);
    for (Eigen::MatrixXd& grad_matrix:connections_gradient_vector_){
        grad_matrix.resize(number_of_robots_,n_dims_);
    }
}

void LaplacianMatrixHandler::InitGradientVector(){
    std::lock_guard<std::mutex> lock(derivatives_mutex_);
    fiedler_gradient_.resize(number_of_robots_*n_dims_);
}

void LaplacianMatrixHandler::UpdateGradientData(
        int robot_index, 
        int conn_index,
        const Eigen::RowVectorXd& newRow
    ){
        std::lock_guard<std::mutex> lock(derivatives_mutex_);
        Eigen::MatrixXd& grad_matrix = connections_gradient_vector_[robot_index];
        grad_matrix.row(conn_index) = newRow;
    }

void LaplacianMatrixHandler::UpdateGradientVector() {
    // 1. Get the Fiedler Value and Vector ONCE
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver;
    Eigen::VectorXd v2;
    
    {
        std::lock_guard<std::mutex> lock(matrix_mutex_);
        solver.compute(laplacian_matrix_);
        if (solver.info() != Eigen::Success) return;
        v2 = solver.eigenvectors().col(1); // The Fiedler Vector
    }

    // 2. Calculate the gradient analytically for each robot i and dimension d
    // The formula: grad = sum_{neighbor j} (v2[i] - v2[j])^2 * (partial weight / partial pos)
    // But since you already have UpdateGradientData (the derivative of the edge weight),
    // we use the chain rule on the Laplacian quadratic form.
    
    std::lock_guard<std::mutex> lock(derivatives_mutex_);
    for (int i = 0; i < number_of_robots_; i++) {
        for (int d = 0; d < n_dims_; d++) {
            double partial_lambda = 0.0;
            
            for (int j = 0; j < number_of_robots_; j++) {
                if (i == j) continue;
                
                // This is the derivative of the weight w_ij with respect to robot i's position
                double dw_ij = connections_gradient_vector_[i](j, d);
                
                // Using the property: v^T * (dL/dp) * v = sum (v_i - v_j)^2 * (dw_ij/dp)
                partial_lambda += std::pow(v2(i) - v2(j), 2) * dw_ij;
            }
            
            fiedler_gradient_[i * n_dims_ + d] = partial_lambda;
        }
    }
}



void LaplacianMatrixHandler::UpdateFiedlerVectorValue(int index, double new_value){
    std::lock_guard<std::mutex> lock(derivatives_mutex_);
    fiedler_gradient_[index] = new_value;
}










void LaplacianMatrixHandler::UpdateInternalMath() {
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
        adjacency_matrix_(i_1, i_2) = new_weight;
        adjacency_matrix_(i_2, i_1) = new_weight;

        UpdateInternalMath();
    }
}



double LaplacianMatrixHandler::GetFiedlerValue() {
    std::lock_guard<std::mutex> lock(matrix_mutex_);
    return GetFiedler(laplacian_matrix_);
}

std::vector<double> LaplacianMatrixHandler::GetGradient(){
    std::lock_guard<std::mutex> lock(derivatives_mutex_);
    return fiedler_gradient_;
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

void LaplacianMatrixHandler::SetLaplacianFromMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->layout.dim.size() < 2) {
        return;
    }

    int rows = msg->layout.dim[0].size;
    int cols = msg->layout.dim[1].size;

    if (msg->data.size() != static_cast<size_t>(rows * cols)) {
        return;
    }

    std::lock_guard<std::mutex> lock(matrix_mutex_);

    laplacian_matrix_.resize(rows, cols);

    int k = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            laplacian_matrix_(i, j) = msg->data[k++];
        }
    }
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