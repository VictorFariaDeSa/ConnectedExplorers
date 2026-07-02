/*******************************************************************************
* Description:


*******************************************************************************/



/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


#include "connected_explorers_interfaces/msg/conn_message.hpp"
#include "connected_explorers_interfaces/msg/swarm_topology.hpp"
#include "connected_explorers_interfaces/msg/robot_pose.hpp"

/*******************************************************************************
* Defines
*******************************************************************************/

// standart values ---
#define INBOX_TOPIC_NAME "inbox"

#define ROBOTS_POSES_TOPIC_NAME "robots_poses"
#define ROBOTS_CONN_READINGS_TOPIC_NAME "robots_conn_readings"


#define QOS_STD_PROFILE 10

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class RobotInboxNode : public rclcpp::Node
{

// parameters ---
private:
    std::string robot_name_prefix_;
    int robot_id_;

// publishers ---
private:
    std::unordered_map<int, rclcpp::Publisher<connected_explorers_interfaces::msg::RobotPose>::SharedPtr> pose_pubs_;
    std::unordered_map<int, rclcpp::Publisher<connected_explorers_interfaces::msg::SwarmTopology>::SharedPtr> adj_pubs_;


// subscribers ---
private:
    rclcpp::Subscription<connected_explorers_interfaces::msg::ConnMessage>::SharedPtr mesh_sub_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr init_timer_;
   
// helpers ---
private:    

// data ---
private:

// mutex ---
private:


/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    RobotInboxNode() : Node("robot_inbox")
    {

        // node parameters ---
        std::string node_param_name;

        node_param_name = "robot_name_prefix";
        this->declare_parameter<std::string>(node_param_name, "robot_");
        robot_name_prefix_ = this->get_parameter(node_param_name).as_string();

        this->declare_parameter<int>("robot_id", 0);
        robot_id_ = this->get_parameter("robot_id").as_int();

        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&RobotInboxNode::init, this)
        );
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init() {
        init_timer_->cancel();

        init_conn_msgs_subcriber();
        
        RCLCPP_INFO(this->get_logger(), "All systems initialized.");
    }

    void init_conn_msgs_subcriber(){
        mesh_sub_ = 
        this->create_subscription<connected_explorers_interfaces::msg::ConnMessage>(
            "robot" + std::to_string(robot_id_) + "/" + INBOX_TOPIC_NAME,
            QOS_STD_PROFILE,
            std::bind(&RobotInboxNode::process_message,this,std::placeholders::_1)
        );
    }

    void process_message(const connected_explorers_interfaces::msg::ConnMessage::SharedPtr msg) {
        int r_id = msg->sender_id;

        // 1. Create publishers if they don't exist yet
        if (pose_pubs_.find(r_id) == pose_pubs_.end())
        {
            std::string pose_topic = "robot" + std::to_string(robot_id_) + "/internal_pose";
            std::string adj_topic  = "robot" + std::to_string(robot_id_) + "/internal_adjacency";

            pose_pubs_[r_id] = this->create_publisher<connected_explorers_interfaces::msg::RobotPose>(pose_topic, 10);
            adj_pubs_[r_id] = this->create_publisher<connected_explorers_interfaces::msg::SwarmTopology>(adj_topic, 10);
            
            RCLCPP_INFO(this->get_logger(), "Rotas criadas para o robot_%d", r_id);
        }

        // 2. Build and publish the RobotPose message
        connected_explorers_interfaces::msg::RobotPose pose_msg;
        pose_msg.robot_id = msg->sender_id; 
        pose_msg.conn_weight = msg->conn_weight;
        pose_msg.pose = msg->pose;
        
        pose_pubs_[r_id]->publish(pose_msg);

        // 3. Build and publish the SwarmTopology message
        connected_explorers_interfaces::msg::SwarmTopology adj_msg;
        adj_msg.robot_id = msg->sender_id;
        adj_msg.conn_weight = msg->conn_weight;
        adj_msg.node_count = msg->node_count;
        adj_msg.adjacency_data = msg->adjacency_data;
        
        adj_pubs_[r_id]->publish(adj_msg);
    }
};


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotInboxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}