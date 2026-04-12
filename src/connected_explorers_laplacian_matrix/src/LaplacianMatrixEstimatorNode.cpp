/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "std_msgs/msg/float64_multi_array.hpp"

/*******************************************************************************
* Defines
*******************************************************************************/

#define  LAPLACIAN_MATRIX_TOPIC_NAME "/laplacian_matrix"
#define QOS_STD_PROFILE 10.0
#define INBOX_TOPIC_NAME "/inbox"

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class LaplacianMatrixEstimator : public rclcpp::Node
{

// parameters ---
private:
    int number_of_robots_;
    int robot_index_; // Remeber this index starts on 1;


    double beta_;
    double dii_;
    // publishers ---
private:

// subscribers ---
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr laplacian_matrix_subscriber_;

// timers ---
private:

// helpers ---
private:

// data ---
private:
    std::vector<double> laplacian_row_;
    std::vector<double> connection_weights_;
// mutex ---
private:
    std::mutex laplacian_row_mutex_;
    std::mutex weights_row_mutex_;

/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    LaplacianMatrixEstimator() : Node("laplacian_estimator") // MODIFY NAME
    {
        // node parameters ---
        std::string node_param_name;

        node_param_name = "number_of_robots";
        this->declare_parameter<int>(node_param_name, 1);
        number_of_robots_ = this->get_parameter(node_param_name).as_int();

        RestartLaplacianRow();
        beta_ = CalculateBeta();
        InitLaplacianSubcriber();

    }

    void InitLaplacianSubcriber(){
        laplacian_matrix_subscriber_ = 
        create_subscription<std_msgs::msg::Float64MultiArray>(
                LAPLACIAN_MATRIX_TOPIC_NAME, 
                QOS_STD_PROFILE,
                std::bind(&LaplacianMatrixEstimator::LaplacianMatrixCallback,this,std::placeholders::_1)
            );
    }

    void InitInboxSubscriber(){

    }


    void LaplacianMatrixCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        int rows = msg->layout.dim[0].size;
        int cols = msg->layout.dim[1].size;
        int target_row = robot_index_-1;
        if (target_row < rows) {
            auto start_it = msg->data.begin() + (target_row * cols);
            auto end_it = start_it + cols;
            {
                std::lock_guard<std::mutex> lock(weights_row_mutex_);
                connection_weights_.assign(start_it, end_it);
            }

            RCLCPP_INFO(this->get_logger(), "Extracted row %d successfully.", target_row);
        }

    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void RestartLaplacianRow(){
        std::lock_guard<std::mutex> lock(laplacian_row_mutex_);
        laplacian_row_.assign(number_of_robots_, 0.0);
        laplacian_row_[robot_index_-1] = 1.0;
    }

    double CalculateBeta(){
        return 1.0/(2*number_of_robots_);
    }
};


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaplacianMatrixEstimator>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}