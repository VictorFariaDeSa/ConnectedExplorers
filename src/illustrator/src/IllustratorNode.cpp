/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

// custom files
#include "illustrator/MarkerDesigner.hpp"
#include "illustrator/ColorClassifier.hpp"

/*******************************************************************************
* DEFINES
*******************************************************************************/

#define QOS_STD_PROFILE 10
#define MARKER_PUBLISH_PERIOD_MS 100

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class IllustratorNode : public rclcpp::Node
{
private:
// parameters ---
    int number_of_robots_;
    std::string robot_name_prefix_;
    std::string reference_map_frame_;
    double line_alpha_;
    double line_scale_;

// publishers ---
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sphere_marker_publisher_;

// subscribers ---
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr laplacian_matrix_subscriber_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> pose_subscriber_list_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr markers_publisher_timer_;

// helpers ---
private:
    std::unique_ptr<MarkerDesigner> designer_; //Estudar
    std::unique_ptr<ColorClassifier> palette_manager_;

// data ---
private:
    std::vector<geometry_msgs::msg::Pose> latest_poses_;

// mutex --
private:
    std::mutex poses_mutex_;



/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    IllustratorNode() : Node("Illustrator")
    {
        // node parameters ---
        std::string node_param_name;

        node_param_name = "reference_frame";
        this->declare_parameter<std::string>(node_param_name, "map");
        reference_map_frame_ = this->get_parameter(node_param_name).as_string();

        node_param_name = "line_alpha";
        this->declare_parameter<double>(node_param_name, 1.0);
        line_alpha_ = this->get_parameter(node_param_name).as_double();


        // helpers init ---
        designer_ = std::make_unique<MarkerDesigner>(reference_map_frame_);

        // publishers and subscribers init ---
        init_marker_publishers();
        init_pose_subscribers();

    }

    ~IllustratorNode(){}

/*******************************************************************************
* Class methods
*******************************************************************************/
private:

    //pusblishers init ---
    void init_marker_publishers(){
        line_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_lines", QOS_STD_PROFILE);
        sphere_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_spheres", QOS_STD_PROFILE);
        
        markers_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(MARKER_PUBLISH_PERIOD_MS), 
            std::bind(&IllustratorNode::marker_publisher_callback,this)
        );
    }

    void marker_publisher_callback(){

    }




    // subscribers init ---
    void init_pose_subscribers(){
        pose_subscriber_list_.reserve(number_of_robots_);
        latest_poses_.resize(number_of_robots_);

        for (int i = 0;i<number_of_robots_;i++){
            std::string topic_name = robot_name_prefix_ + std::to_string(i) + "/position"; //TODO review this position topic name
            
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription = 
            this->create_subscription<geometry_msgs::msg::Pose>(
                topic_name, 
                QOS_STD_PROFILE,
                [this, i](const geometry_msgs::msg::Pose::SharedPtr msg) {
                    this->pose_subscriber_callback(msg, i);
                }
            );

            pose_subscriber_list_.push_back(subscription);
        }
    }

    // subscribers callback functions ---
    void pose_subscriber_callback(const geometry_msgs::msg::Pose::SharedPtr msg, int robot_index){
        std::lock_guard<std::mutex> lock(poses_mutex_);
        latest_poses_[robot_index] = *msg;
    }







    


};



/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IllustratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}