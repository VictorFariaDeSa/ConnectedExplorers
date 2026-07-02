/*******************************************************************************
* Description:
* Node responsible for gathering internal robot data (pose, adjacency) 
* and broadcasting it to the external mesh network via the outbox.
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "connected_explorers_interfaces/msg/conn_message.hpp"

/*******************************************************************************
* Defines
*******************************************************************************/

// standard values ---
#define OUTBOX_TOPIC_NAME "outbox"
#define LAPLACIAN_GUESS_TOPIC_NAME "laplacian_guess"
#define NEIGH_POSE_TOPIC "neigh_pose"
#define NEIGH_ADJ_TOPIC "neigh_adjacency"

#define INTERNAL_POSE_TOPIC "position"
#define INTERNAL_ADJ_TOPIC "know_connections"

#define QOS_STD_PROFILE 10

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class RobotOutboxNode : public rclcpp::Node
{

// parameters ---
private:
    int robot_id_;
    double publish_rate_;

// publishers ---
private:
    rclcpp::Publisher<connected_explorers_interfaces::msg::ConnMessage>::SharedPtr mesh_pub_;

// subscribers ---
private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr adj_sub_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr publish_timer_;
   
// data cache ---
private:
    geometry_msgs::msg::Pose latest_pose_;
    std::vector<double> latest_adj_data_;
    bool has_pose_ = false;
    bool has_adj_ = false;

// mutex ---
private:
    std::mutex data_mutex_;

/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    RobotOutboxNode() : Node("robot_outbox")
    {
        // node parameters ---
        this->declare_parameter<int>("robot_id", 0);
        this->declare_parameter<double>("publish_rate", 10.0); // Hz

        robot_id_ = this->get_parameter("robot_id").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();

        init();
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init() {
        // Init Publisher
        mesh_pub_ = this->create_publisher<connected_explorers_interfaces::msg::ConnMessage>(
            "robot" + std::to_string(robot_id_) + "/" + OUTBOX_TOPIC_NAME, 
            QOS_STD_PROFILE
        );

        // Init Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot" + std::to_string(robot_id_) + "/" + INTERNAL_POSE_TOPIC,
            QOS_STD_PROFILE,
            std::bind(&RobotOutboxNode::pose_callback, this, std::placeholders::_1)
        );

        adj_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "robot" + std::to_string(robot_id_) + "/" + INTERNAL_ADJ_TOPIC,
            QOS_STD_PROFILE,
            std::bind(&RobotOutboxNode::adj_callback, this, std::placeholders::_1)
        );

        // Init Timer
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        publish_timer_ = this->create_wall_timer(
            timer_period, 
            std::bind(&RobotOutboxNode::publish_outbox_message, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Outbox initialized for robot_%d.", robot_id_);
    }

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_pose_ = *msg;
        has_pose_ = true;
    }

    void adj_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_adj_data_ = msg->data;
        has_adj_ = true;
    }

    void publish_outbox_message() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!has_pose_ || !has_adj_) {
            return;
        }

        for (size_t i = 0; i < latest_adj_data_.size(); ++i) {
            double weight = latest_adj_data_[i];

            // Check if the connection is strong enough
            if (weight > 0.1) {
                connected_explorers_interfaces::msg::ConnMessage out_msg;
                
                out_msg.sender_id = robot_id_;
                out_msg.receiver_id = i + 1; // Assuming the index 'i' maps to robot_id (e.g., index 0 is robot 1)
                out_msg.conn_weight = weight;
                out_msg.pose = latest_pose_;
                out_msg.adjacency_data = latest_adj_data_;

                // Publish a dedicated message for this specific neighbor
                mesh_pub_->publish(out_msg);
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
    auto node = std::make_shared<RobotOutboxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}