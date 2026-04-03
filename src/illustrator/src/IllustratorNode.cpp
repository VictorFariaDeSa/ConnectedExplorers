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
#include "my_robot_interfaces/msg/robot_task.hpp"

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
    std::vector<rclcpp::Subscription<my_robot_interfaces::msg::RobotTask>::SharedPtr> role_subscriber_list_;

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
    std_msgs::msg::Float64MultiArray latest_laplacian_matrix_;
    std::vector<my_robot_interfaces::msg::RobotTask> latest_robots_roles_;
// mutex --
private:
    std::mutex poses_mutex_;
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
        RCLCPP_INFO(this->get_logger(), "Illustrator initialized for %d robots", number_of_robots_);
        node_param_name = "robot_name_prefix";
        this->declare_parameter<std::string>(node_param_name, "robot_");
        robot_name_prefix_ = this->get_parameter(node_param_name).as_string();

        node_param_name = "reference_frame";
        this->declare_parameter<std::string>(node_param_name, "map");
        reference_map_frame_ = this->get_parameter(node_param_name).as_string();

        node_param_name = "line_alpha";
        this->declare_parameter<double>(node_param_name, 1.0);
        line_alpha_ = this->get_parameter(node_param_name).as_double();

        node_param_name = "line_scale";
        this->declare_parameter<double>(node_param_name, 0.05);
        line_scale_ = this->get_parameter(node_param_name).as_double();


        // helpers init ---
        designer_ = std::make_unique<MarkerDesigner>(reference_map_frame_);
        palette_manager_ = std::make_unique<ColorClassifier>();

        resize_vectors();

        // publishers and subscribers init ---
        init_marker_publishers();
        init_pose_subscribers();
        init_role_subscribers();
        init_laplacian_matrix_subscriber();
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
        rclcpp::Time now = this->get_clock()->now();
        std::string line_ns = "lines";
        std::string sphere_ns = "spheres";
        std::scoped_lock lock(poses_mutex_, laplacian_matrix_mutex_, roles_mutex_);
        
        if (latest_poses_.empty() || latest_laplacian_matrix_.data.empty()) {
            return; 
        }

        visualization_msgs::msg::Marker line_marker = designer_->GetBaseLineMarkers(now, line_ns, line_scale_);
        visualization_msgs::msg::Marker sphere_marker = designer_->GetBaseSphereMarkers(now, sphere_ns, 0.2);

        for (int i = 0; i < number_of_robots_; i++){
            geometry_msgs::msg::Point p_i = latest_poses_[i].position;
            std_msgs::msg::ColorRGBA color_i = palette_manager_->GetColorByTask(latest_robots_roles_[i].current_task);
            designer_->AddSphereToMarkerMsg(sphere_marker, p_i, color_i);
            for (int j = i + 1; j < number_of_robots_; j++) {
                int matrix_idx = (i * number_of_robots_) + j;
                geometry_msgs::msg::Point p_j = latest_poses_[j].position;
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
    void init_pose_subscribers(){
        for (int i = 0;i<number_of_robots_;i++){
            std::string topic_name = robot_name_prefix_ + std::to_string(i+1) + "/position"; //TODO review this position topic name

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

    void init_role_subscribers(){
        for (int i = 0;i<number_of_robots_;i++){
            std::string topic_name = robot_name_prefix_ + std::to_string(i+1) + "/role"; //TODO review this role topic name
            
            rclcpp::Subscription<my_robot_interfaces::msg::RobotTask>::SharedPtr subscription = 
            this->create_subscription<my_robot_interfaces::msg::RobotTask>(
                topic_name, 
                QOS_STD_PROFILE,
                [this, i](const my_robot_interfaces::msg::RobotTask::SharedPtr msg) {
                    this->role_subscriber_callback(msg, i);
                }
            );

            role_subscriber_list_.push_back(subscription);
        }
    }


    void init_laplacian_matrix_subscriber(){
        laplacian_matrix_subscriber_ = 
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "laplacian_matrix",
            QOS_STD_PROFILE,
            std::bind(&IllustratorNode::laplacian_matrix_subscriber_callback,this,std::placeholders::_1)
        );
    }




    // subscribers callback functions ---
    void pose_subscriber_callback(const geometry_msgs::msg::Pose::SharedPtr msg, int robot_index){
        std::lock_guard<std::mutex> lock(poses_mutex_);
        latest_poses_[robot_index] = *msg;
    }

    void role_subscriber_callback(const my_robot_interfaces::msg::RobotTask::SharedPtr msg, int robot_index){
        std::lock_guard<std::mutex> lock(roles_mutex_);
        latest_robots_roles_[robot_index] = *msg;
    }

    void laplacian_matrix_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        std::lock_guard<std::mutex> lock(laplacian_matrix_mutex_);
        latest_laplacian_matrix_ = *msg;
    }





private:
    void resize_vectors(){
        pose_subscriber_list_.reserve(number_of_robots_);
        latest_poses_.resize(number_of_robots_);
        
        my_robot_interfaces::msg::RobotTask default_role;
        default_role.current_task = my_robot_interfaces::msg::RobotTask::IDLE;
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