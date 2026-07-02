/*******************************************************************************
* Description:

- GlobalSupervisorNode

This Node is responsable for keeping the environment information and simulate 
plausible communication between the robots

*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

// custom messages
#include "connected_explorers_interfaces/msg/line_clearance_array.hpp"

// custom files
#include "connected_explorers_utils/MultiRobotsPoseHandler.hpp"
#include "connected_explorers_laplacian_matrix/LaplacianMatrixHandler.hpp"

/*******************************************************************************
* Defines
*******************************************************************************/

// standard values ---
#define ROBOTS_POSE_TOPIC_NAME "/position"
#define MAP_TOPIC_NAME "/map"
#define CONN_WEIGHT_TOPIC_NAME "line_clearance"
#define LAPLACIAN_MATRIX_TOPIC_NAME "laplacian_matrix"
#define FIEDLER_VALUE_TOPIC_NAME "fiedler_value"
#define FIEDLER_GRADIENT_TOPIC_NAME "lambda2_gradient"
#define KNOW_CONNECTIONS_TOPIC_NAME "know_connections"

#define LAPLACIAN_MATRIX_PUBLISHER_PERIOD_MS 100
#define FIEDLER_GRADIENT_PUBLISHER_PERIOD_MS 100
#define INBOX_PUBLISHER_PERIOD_MS 100

#define QOS_STD_PROFILE 10

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class GlobalSupervisorNode : public rclcpp::Node
{

// parameters ---
private:
    int number_of_robots_;
    int problem_dimension_;
    std::string robot_name_prefix_;
    std::string laplacian_matrix_topic_name_;
    int laplacian_matrix_publish_period_ms_;
    int fiedler_gradient_publish_period_ms_;

// publishers ---
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr laplacian_matrix_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fiedler_value_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fiedler_gradient_publisher_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> robot_know_connection_weight_publisher_list_;

// subscribers ---
private:
    rclcpp::Subscription<connected_explorers_interfaces::msg::LineClearanceArray>::SharedPtr line_clearance_subscriber_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr laplacian_matrix_publisher_timer_;
    rclcpp::TimerBase::SharedPtr fiedler_gradient_publisher_timer_;
    std::vector<rclcpp::TimerBase::SharedPtr> inbox_timers_;

// helpers ---
private:    
    std::unique_ptr<connected_explorers_utils::LaplacianMatrixHandler> laplacian_matrix_handler_;

// data ---
private:

// mutex ---
private:
    std::mutex data_mutex_;


/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    GlobalSupervisorNode() : Node("global_supervisor")
    {

        // node parameters ---
        std::string node_param_name;

        node_param_name = "number_of_robots";
        this->declare_parameter<int>(node_param_name, 1);
        number_of_robots_ = this->get_parameter(node_param_name).as_int();

        node_param_name = "robot_name_prefix";
        this->declare_parameter<std::string>(node_param_name, "robot_");
        robot_name_prefix_ = this->get_parameter(node_param_name).as_string();

        node_param_name = "problem_dimension";
        this->declare_parameter<int>(node_param_name, 2);
        problem_dimension_ = this->get_parameter(node_param_name).as_int();


        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&GlobalSupervisorNode::init, this)
        );
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init() {
        init_timer_->cancel();

        laplacian_matrix_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(LAPLACIAN_MATRIX_TOPIC_NAME, QOS_STD_PROFILE);
        fiedler_value_publisher_ = create_publisher<std_msgs::msg::Float64>(FIEDLER_VALUE_TOPIC_NAME, QOS_STD_PROFILE);
        fiedler_gradient_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(FIEDLER_GRADIENT_TOPIC_NAME, QOS_STD_PROFILE);
        
        for (int robot_index = 0; robot_index < number_of_robots_; robot_index++){
            std::string topic_name = robot_name_prefix_ + std::to_string(robot_index+1) + "/" + KNOW_CONNECTIONS_TOPIC_NAME;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher = 
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                topic_name, 
                QOS_STD_PROFILE
            );
            robot_know_connection_weight_publisher_list_.push_back(publisher);
        }

        line_clearance_subscriber_ = create_subscription<connected_explorers_interfaces::msg::LineClearanceArray>(
            CONN_WEIGHT_TOPIC_NAME,
            QOS_STD_PROFILE,
            std::bind(&GlobalSupervisorNode::LineClearanceCallback, this, std::placeholders::_1)
        );

        laplacian_matrix_handler_ = std::make_unique<connected_explorers_utils::LaplacianMatrixHandler>(
            number_of_robots_,
            problem_dimension_
        );

        StartLaplacianMatrixPublisher();
        StartFiedlerGradientPublisher();
        StartKnowConnPublishers();
        
        RCLCPP_INFO(this->get_logger(), "All systems initialized.");
    }

    void StartLaplacianMatrixPublisher() {
        laplacian_matrix_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(LAPLACIAN_MATRIX_PUBLISHER_PERIOD_MS), 
            std::bind(&GlobalSupervisorNode::PublishLaplacianData, this));
    }

    void StartFiedlerGradientPublisher(){
        fiedler_gradient_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(FIEDLER_GRADIENT_PUBLISHER_PERIOD_MS), 
            std::bind(&GlobalSupervisorNode::PublishFiedlerGradient, this));
    }

    void StartKnowConnPublishers() {
        for (int robot_index = 0; robot_index < number_of_robots_; robot_index++){
            auto timer = this->create_wall_timer(
                std::chrono::milliseconds(INBOX_PUBLISHER_PERIOD_MS), 
                [this, robot_index]() {
                    this->PublishKnowConnData(robot_index);
                }
            );
            inbox_timers_.push_back(timer);
        }
    }

    void LineClearanceCallback(const connected_explorers_interfaces::msg::LineClearanceArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (connected_explorers_interfaces::msg::LineClearance conn:msg->clearances){
            laplacian_matrix_handler_->UpdateConnWeight(conn.weight,conn.robot1_id,conn.robot2_id);
            Eigen::RowVectorXd d1(3);
            d1 << conn.dx1, conn.dy1, conn.dz1;   
            laplacian_matrix_handler_->UpdateGradientData(conn.robot1_id,conn.robot2_id,d1);
            Eigen::RowVectorXd d2(3);
            d2 << conn.dx2, conn.dy2, conn.dz2;   
            laplacian_matrix_handler_->UpdateGradientData(conn.robot2_id,conn.robot1_id,d2);
        }
    }

    void PublishLaplacianData() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        laplacian_matrix_publisher_->publish(laplacian_matrix_handler_->GetLaplacianMsg());

        std_msgs::msg::Float64 fiedler_msg;
        fiedler_msg.data = laplacian_matrix_handler_->GetFiedlerValue();
        fiedler_value_publisher_->publish(fiedler_msg);
    }

    void PublishFiedlerGradient(){
        std::vector<double> gradient;
        int vec_size = 0;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            laplacian_matrix_handler_->UpdateGradientVector();
            gradient = laplacian_matrix_handler_->GetGradient();
            vec_size = gradient.size();
        }

        if (vec_size == 0) return;

        auto msg = std_msgs::msg::Float64MultiArray();

        std_msgs::msg::MultiArrayDimension dim_rows;
        dim_rows.label = "rows";
        dim_rows.size = vec_size;
        dim_rows.stride = vec_size;
        msg.layout.dim.push_back(dim_rows);

        std_msgs::msg::MultiArrayDimension dim_cols;
        dim_cols.label = "cols";
        dim_cols.size = 1;
        dim_cols.stride = 1;
        msg.layout.dim.push_back(dim_cols);

        msg.data = gradient; 

        fiedler_gradient_publisher_->publish(msg);
    }

    void PublishKnowConnData(int index){
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher = robot_know_connection_weight_publisher_list_[index];
        Eigen::MatrixXd adjancy_matrix = laplacian_matrix_handler_->GetAdjacencyMatrix();
        std_msgs::msg::Float64MultiArray msg;
        Eigen::VectorXd target_row = adjancy_matrix.row(index);
        msg.data.assign(target_row.data(), target_row.data() + target_row.size());
        publisher->publish(msg);
    }

};

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalSupervisorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}