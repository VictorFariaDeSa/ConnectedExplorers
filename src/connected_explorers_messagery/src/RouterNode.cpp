/*******************************************************************************
* Description:
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "std_msgs/msg/float64_multi_array.hpp"

// custom messages
#include "connected_explorers_interfaces/msg/conn_message.hpp"

//custom files
#include "connected_explorers_laplacian_matrix/LaplacianMatrixHandler.hpp"


/*******************************************************************************
* Defines
*******************************************************************************/

// standard values ---

#define LAPLACIAN_MATRIX_TOPIC "/laplacian_matrix"
#define INBOX_TOPIC_PREFIX "/inbox"
#define OUTBOX_TOPIC_PREFIX "/outbox"

#define QOS_STD_PROFILE 10

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class RouterNode : public rclcpp::Node
{

// parameters ---
private:
    int number_of_robots_;
    int problem_dimension_;

// publishers ---
private:
    std::unordered_map<int,rclcpp::Publisher<connected_explorers_interfaces::msg::ConnMessage>::SharedPtr> conn_message_publisher_map_;


// subscribers ---
private:
    // se inscreve em todos os tópicos de envio dos outbox (deve enviar a mensagem e o id) (esse cara transforma o ide no topico de inbox)
    // se isncreve no tópico real de matriz laplacina para filtrar envios invisiveis
    std::unordered_map<int,rclcpp::Subscription<connected_explorers_interfaces::msg::ConnMessage>::SharedPtr> conn_message_subscriber_map_; 
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
    RouterNode() : Node("message_router"){

                // node parameters ---
        std::string node_param_name;

        node_param_name = "number_of_robots";
        this->declare_parameter<int>(node_param_name, 1);
        number_of_robots_ = this->get_parameter(node_param_name).as_int();

        node_param_name = "problem_dimension";
        this->declare_parameter<int>(node_param_name, 2);
        problem_dimension_ = this->get_parameter(node_param_name).as_int();


        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&RouterNode::init, this)
        );
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init() {
        init_timer_->cancel();

        laplacian_matrix_handler_ = std::make_unique<connected_explorers_utils::LaplacianMatrixHandler>(
            number_of_robots_,
            problem_dimension_
        );

        laplacian_matrix_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            LAPLACIAN_MATRIX_TOPIC,
            QOS_STD_PROFILE,
            std::bind(&RouterNode::laplacian_matrix_callback, this, std::placeholders::_1)                
        );
        init_message_subscribers();
        RCLCPP_INFO(this->get_logger(), "All systems initialized.");
    }


    void init_message_subscribers() {
        for (int i = 1; i<= number_of_robots_;i++){
            std::string outbox_complete_topic = "robot" + std::to_string(i) + OUTBOX_TOPIC_PREFIX;
            conn_message_subscriber_map_[i] = this->create_subscription<connected_explorers_interfaces::msg::ConnMessage>(
                outbox_complete_topic, 
                QOS_STD_PROFILE,
                std::bind(&RouterNode::forward_message, this, std::placeholders::_1)                
            );
        }
    }


    void forward_message(const connected_explorers_interfaces::msg::ConnMessage::SharedPtr msg){
        if (msg->conn_weight > 0.1){
            int r_id = msg->receiver_id;
            if (conn_message_publisher_map_.find(r_id) == conn_message_publisher_map_.end())
            {
                std::string inbox_complete_topic = "robot" + std::to_string(r_id) + INBOX_TOPIC_PREFIX;
                conn_message_publisher_map_[r_id] = this->create_publisher<connected_explorers_interfaces::msg::ConnMessage>(inbox_complete_topic, 10);
                
                RCLCPP_INFO(this->get_logger(), "Router Node inbox publisher created for robot_%d", r_id);
            }
            conn_message_publisher_map_[r_id]->publish(*msg);
        }
    }

    void laplacian_matrix_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        laplacian_matrix_handler_->SetLaplacianFromMsg(msg);
    }







};
/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RouterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}