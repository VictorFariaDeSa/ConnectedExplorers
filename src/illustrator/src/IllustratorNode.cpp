/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

// custom messages
#include "connected_explorers_interfaces/msg/robot_task.hpp"

// custom files
#include "illustrator/MarkerDesigner.hpp"
#include "illustrator/ColorClassifier.hpp"
#include "connected_explorers_utils/MultiRobotsPoseHandler.hpp"

/*******************************************************************************
* DEFINES
*******************************************************************************/

#define QOS_STD_PROFILE 10
#define MARKER_PUBLISH_PERIOD_MS 100

#define POSE_TOPIC_NAME "/position"
#define ROLE_TOPIC_NAME "/role"
#define LAPLACIAN_MATRIX_TOPIC_NAME "/laplacian_matrix"


// line configs ---
#define LINE_MARKER_TOPIC_NAME "marker_lines"
#define LINE_DEFAULT_SCALE 0.2
#define LINE_DEFAULT_ALPHA 1.0

// sphere configs ---
#define SPHERE_MARKER_TOPIC_NAME "marker_spheres"
#define SPEHRE_DEFAULT_DIAMETER 0.2
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

// publishers ---
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sphere_marker_publisher_;

// subscribers ---
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr laplacian_matrix_subscriber_;
    std::vector<rclcpp::Subscription<connected_explorers_interfaces::msg::RobotTask>::SharedPtr> role_subscriber_list_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr markers_publisher_timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;

// helpers ---
private:
    std::unique_ptr<MarkerDesigner> designer_; //Estudar
    std::unique_ptr<ColorClassifier> palette_manager_;
    std::unique_ptr<connected_explorers_utils::MultiRobotsPoseHandler> pose_handler_;
// data ---
private:
    std_msgs::msg::Float64MultiArray latest_laplacian_matrix_;
    std::vector<connected_explorers_interfaces::msg::RobotTask> latest_robots_roles_;
// mutex --
private:
    std::mutex laplacian_matrix_mutex_;
    std::mutex roles_mutex_;



/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    IllustratorNode() : Node("Illustrator")
    {
        // node parameters ---
        std::string node_param_name;

        node_param_name = "number_of_robots";
        this->declare_parameter<int>(node_param_name, 1);
        number_of_robots_ = this->get_parameter(node_param_name).as_int();

        node_param_name = "robot_name_prefix";
        this->declare_parameter<std::string>(node_param_name, "robot_");
        robot_name_prefix_ = this->get_parameter(node_param_name).as_string();

        node_param_name = "reference_frame";
        this->declare_parameter<std::string>(node_param_name, "map");
        reference_map_frame_ = this->get_parameter(node_param_name).as_string();  
        
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&IllustratorNode::init, this)
        );
    }

    ~IllustratorNode(){}

/*******************************************************************************
* Class methods
*******************************************************************************/
private:

    void init() {
        init_timer_->cancel();
        
        designer_ = std::make_unique<MarkerDesigner>(reference_map_frame_);
        palette_manager_ = std::make_unique<ColorClassifier>();
        pose_handler_ = std::make_unique<connected_explorers_utils::MultiRobotsPoseHandler>(
            this->shared_from_this(),
            number_of_robots_,
            robot_name_prefix_,
            POSE_TOPIC_NAME,
            QOS_STD_PROFILE
        );

        pose_handler_->InitPoseSubscribers();
        
        resize_vectors();
        init_marker_publishers();
        init_role_subscribers();
        init_laplacian_matrix_subscriber();
        
        RCLCPP_INFO(this->get_logger(), "All systems initialized.");
    }


    //pusblishers init ---
    void init_marker_publishers(){
        line_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(LINE_MARKER_TOPIC_NAME, QOS_STD_PROFILE);
        sphere_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(SPHERE_MARKER_TOPIC_NAME, QOS_STD_PROFILE);
        
        markers_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(MARKER_PUBLISH_PERIOD_MS), 
            std::bind(&IllustratorNode::marker_publisher_callback,this)
        );
    }

    void marker_publisher_callback(){
        rclcpp::Time now = this->get_clock()->now();
        std::string line_ns = "lines";
        std::string sphere_ns = "spheres";
        std::scoped_lock lock(laplacian_matrix_mutex_, roles_mutex_);
        std::vector<geometry_msgs::msg::Pose> robots_poses = pose_handler_->GetRobotsPoses();

        if (robots_poses.empty() || latest_laplacian_matrix_.data.empty()) {
            return; 
        }

        visualization_msgs::msg::Marker line_marker = designer_->GetBaseLineMarkers(now, line_ns, LINE_DEFAULT_SCALE);
        visualization_msgs::msg::Marker sphere_marker = designer_->GetBaseSphereMarkers(now, sphere_ns, SPEHRE_DEFAULT_DIAMETER);

        for (int i = 0; i < number_of_robots_; i++){
            geometry_msgs::msg::Point p_i = robots_poses[i].position;
            std_msgs::msg::ColorRGBA color_i = palette_manager_->GetColorByTask(latest_robots_roles_[i].current_task);
            designer_->AddSphereToMarkerMsg(sphere_marker, p_i, color_i);
            for (int j = i + 1; j < number_of_robots_; j++) {
                int matrix_idx = (i * number_of_robots_) + j;
                geometry_msgs::msg::Point p_j = robots_poses[j].position;
                float score = latest_laplacian_matrix_.data[matrix_idx];
                std_msgs::msg::ColorRGBA line_color = palette_manager_->GetColorByScore(score);
                designer_->AddLineToMarkerMsg(
                    line_marker, 
                    p_i, 
                    p_j, 
                    line_color
                );
            }
        }
        line_marker_publisher_->publish(line_marker);
        sphere_marker_publisher_->publish(sphere_marker);
    }




    // subscribers init ---
    void init_role_subscribers(){
        for (int i = 0;i<number_of_robots_;i++){
            std::string topic_name = robot_name_prefix_ + std::to_string(i+1) + ROLE_TOPIC_NAME; //TODO review this role topic name
            
            rclcpp::Subscription<connected_explorers_interfaces::msg::RobotTask>::SharedPtr subscription = 
            this->create_subscription<connected_explorers_interfaces::msg::RobotTask>(
                topic_name, 
                QOS_STD_PROFILE,
                [this, i](const connected_explorers_interfaces::msg::RobotTask::SharedPtr msg) {
                    this->role_subscriber_callback(msg, i);
                }
            );

            role_subscriber_list_.push_back(subscription);
        }
    }


    void init_laplacian_matrix_subscriber(){
        laplacian_matrix_subscriber_ = 
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            LAPLACIAN_MATRIX_TOPIC_NAME,
            QOS_STD_PROFILE,
            std::bind(&IllustratorNode::laplacian_matrix_subscriber_callback,this,std::placeholders::_1)
        );
    }


    // callbacks definitions
    void role_subscriber_callback(const connected_explorers_interfaces::msg::RobotTask::SharedPtr msg, int robot_index){
        std::lock_guard<std::mutex> lock(roles_mutex_);
        latest_robots_roles_[robot_index] = *msg;
    }

    void laplacian_matrix_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        std::lock_guard<std::mutex> lock(laplacian_matrix_mutex_);
        latest_laplacian_matrix_ = *msg;
    }





private:
    void resize_vectors(){
        connected_explorers_interfaces::msg::RobotTask default_role;
        default_role.current_task = connected_explorers_interfaces::msg::RobotTask::BASE;
        latest_robots_roles_.resize(number_of_robots_, default_role);
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