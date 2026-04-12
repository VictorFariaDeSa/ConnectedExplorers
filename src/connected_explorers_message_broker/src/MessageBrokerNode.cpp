/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "std_msgs/msg/float64_multi_array.hpp"

//custom messages ---
#include "connected_explorers_interfaces/msg/sync_state.hpp"

// custom files
#include "connected_explorers_utils/LaplacianMatrixHandler.hpp"

/*******************************************************************************
* Defines
*******************************************************************************/

#define QOS_STD_PROFILE 10
#define BROADCAST_LISTENER_TOPIC_NAME "/broadcast"
#define ROBOTS_INBOX_TOPIC_NAME "/inbox"
#define CONNECTION_THRESHOLD_VALUE 1.0
#define LAPLACIAN_MATRIX_TOPIC "/laplacian_matrix"

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class MessageBrokerNode : public rclcpp::Node
{

// parameters ---
private:
    int number_of_robots_;
    std::string robot_name_prefix_;

// publishers ---
private:
    std::vector<rclcpp::Publisher<connected_explorers_interfaces::msg::SyncState>::SharedPtr> inbox_publishers_list_;

// subscribers ---
private:
    std::vector<rclcpp::Subscription<connected_explorers_interfaces::msg::SyncState>::SharedPtr> broadcast_subscribers_list_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr laplacian_matrix_subscriber_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr init_timer_;

// helpers ---
private:
    std::unique_ptr<connected_explorers_utils::LaplacianMatrixHandler> laplacian_matrix_handler_;

// data ---
private:

// mutex ---
private:

/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    MessageBrokerNode() : Node("message_broker")
    {
        // node parameters ---
        std::string node_param_name;

        node_param_name = "number_of_robots";
        this->declare_parameter<int>(node_param_name, 1);
        number_of_robots_ = this->get_parameter(node_param_name).as_int();

        node_param_name = "robot_name_prefix";
        this->declare_parameter<std::string>(node_param_name, "robot_");
        robot_name_prefix_ = this->get_parameter(node_param_name).as_string();

        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&MessageBrokerNode::init, this)
        );
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init(){
        init_timer_->cancel();

        laplacian_matrix_handler_ = std::make_unique<connected_explorers_utils::LaplacianMatrixHandler>(
            number_of_robots_
        );
        
        InitBroadcastSubscribers();
        InitInboxPublishers();
        InitLaplacianMatrixSubscriber();
    }

    void InitLaplacianMatrixSubscriber(){
        laplacian_matrix_subscriber_=this->
        create_subscription<std_msgs::msg::Float64MultiArray>(
            LAPLACIAN_MATRIX_TOPIC,
            QOS_STD_PROFILE,
            std::bind(&MessageBrokerNode::LaplacianMatrixCallback,this,std::placeholders::_1)
        );
    }

    void InitBroadcastSubscribers(){
        broadcast_subscribers_list_.resize(number_of_robots_);
        for (int i = 0;i<number_of_robots_;i++){
            std::string topic_name = robot_name_prefix_ + std::to_string(i+1) + BROADCAST_LISTENER_TOPIC_NAME;

            broadcast_subscribers_list_[i] = this->
            create_subscription<connected_explorers_interfaces::msg::SyncState>(
                topic_name, 
                QOS_STD_PROFILE,
                [this, i](const connected_explorers_interfaces::msg::SyncState::SharedPtr msg) {
                    this->BroadcastSubscriberCallback(msg, i);
                }
            );
        }
    }

    void InitInboxPublishers(){
        inbox_publishers_list_.resize(number_of_robots_);
        for (int i = 0;i<number_of_robots_;i++){
            std::string topic_name = robot_name_prefix_ + std::to_string(i+1) + ROBOTS_INBOX_TOPIC_NAME;
            
            inbox_publishers_list_[i]= this->
            create_publisher<connected_explorers_interfaces::msg::SyncState>(topic_name, QOS_STD_PROFILE);
        }
    }


    void LaplacianMatrixCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        laplacian_matrix_handler_->SetLaplacianFromMsg(msg);
    }

    void BroadcastSubscriberCallback(const connected_explorers_interfaces::msg::SyncState::SharedPtr msg, int robot_index){
        Eigen::MatrixXd current_laplacian_matrix = laplacian_matrix_handler_->GetLaplacianMatrix();
        
        if (current_laplacian_matrix.rows() <= robot_index || current_laplacian_matrix.cols() < number_of_robots_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Laplacian matrix not yet received or size mismatch!");
            return;
        }

        for (int i=0;i<number_of_robots_;i++){
            if (robot_index==i){
                continue;
            }
            double conn_weight = current_laplacian_matrix(robot_index,i);
            if (conn_weight > CONNECTION_THRESHOLD_VALUE){
                inbox_publishers_list_[i]->publish(*msg);
            }
        }
    }

};


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessageBrokerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}