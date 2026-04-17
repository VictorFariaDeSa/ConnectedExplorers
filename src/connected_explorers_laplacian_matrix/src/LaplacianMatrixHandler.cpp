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

// URGENT CHANGE THIS ESTIMATION METHOD !!!!!!
double LaplacianMatrixHandler::EstimateFiedlerFutureValue(int robot_index, int direction) {
    Eigen::MatrixXd temp_laplacian;
    {
        std::lock_guard<std::mutex> lock(matrix_mutex_);
        temp_laplacian = laplacian_matrix_;
    }
    Eigen::VectorXd weight_deltas = connections_gradient_vector_[robot_index].col(direction);

    for (int neighbor_idx = 0; neighbor_idx < number_of_robots_; ++neighbor_idx) {
        if (neighbor_idx == robot_index) continue;

        double dw = weight_deltas(neighbor_idx);
        temp_laplacian(robot_index, neighbor_idx) -= dw;
        temp_laplacian(neighbor_idx, robot_index) -= dw;

        temp_laplacian(robot_index, robot_index) += dw;
        temp_laplacian(neighbor_idx, neighbor_idx) += dw;
    }

    return GetFiedler(temp_laplacian);
}

void LaplacianMatrixHandler::UpdateFiedlerVectorValue(int index, double new_value){
    std::lock_guard<std::mutex> lock(derivatives_mutex_);
    fiedler_gradient_[index] = new_value;
}

void LaplacianMatrixHandler::UpdateGradientVector(){
    double curr_fiedler = GetFiedlerValue();
    for (int i=0;i<number_of_robots_;i++){
        for (int d=0;d<n_dims_;d++){
            double new_fiedler = EstimateFiedlerFutureValue(i,d);
            double delta = new_fiedler - curr_fiedler;
            UpdateFiedlerVectorValue(i*n_dims_+d,delta);
        }
    }
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