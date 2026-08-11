/*******************************************************************************
* Includes
*******************************************************************************/
#include "rclcpp/rclcpp.hpp"

// messages
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// custom messages
#include "connected_explorers_interfaces/msg/swarm_topology.hpp"
#include "connected_explorers_interfaces/msg/robot_pose.hpp"

// custom files
#include "connected_explorers_laplacian_matrix/LaplacianMatrixHandler.hpp"
#include "connected_explorers_utils/MapHandler3d.hpp"
#include "connected_explorers_utils/MapHandler.hpp" // Added 2D MapHandler
#include "connected_explorers_utils/ConnWeightHandler.hpp"

#include <chrono>
#include <mutex>
#include <unordered_map>
#include <array>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

/*******************************************************************************
* Defines
*******************************************************************************/
#define CONN_CORRECTION_STEP 0.8
#define POSE_CORRETCTION_STEP 0.1
#define QOS_STD_PROFILE 10.0
#define MAP_RESOLUTION 0.1 

#define LAPLACIAN_GUESS_TOPIC "laplacian_guess"
#define FIEDLER_GRADIENT_GUESS_TOPIC "gradient_guess"
#define FIEDLER_GUESS_TOPIC "fiedler_guess"
#define NEIGH_POSE_TOPIC "internal_pose"
#define NEIGH_ADJ_TOPIC "internal_adjacency"
#define CONN_WEIGHT_TOPIC "know_connections"
#define POSE_TOPIC "position"
#define POINT_CLOUD_TOPIC "/octomap_point_cloud_centers"
#define MAP_TOPIC "map" // Added for 2D Occupancy Grid

#define LOS_ALPHA -6.0
#define LOS_BETA 0.5
#define DISTANCE_ALPHA 1.0
#define DISTANCE_BETA 6.0

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/
class LaplacianMatrixEstimator : public rclcpp::Node
{
// parameters ---
private:
    int number_of_robots_;
    int robot_index_;
    int problem_dimensions_;

    double los_alpha_;
    double los_beta_;
    double distance_alpha_;
    double distance_beta_;


// publishers ---
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr laplacian_matrix_guess_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fiedler_guess_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fiedler_gradient_guess_publisher_;
    std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> guess_position_publishers_;

// subscribers ---
private:
    rclcpp::Subscription<connected_explorers_interfaces::msg::SwarmTopology>::SharedPtr neigh_conn_subscriber_;
    rclcpp::Subscription<connected_explorers_interfaces::msg::RobotPose>::SharedPtr neigh_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr own_conn_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr own_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

// helpers ---
private:
    std::unique_ptr<connected_explorers_utils::LaplacianMatrixHandler> laplacian_matrix_handler_;
    std::unique_ptr<connected_explorers_utils::MapHandler3d> map_handler_3d_;
    std::shared_ptr<connected_explorers_utils::MapHandler> map_handler_2d_; // Added 2D Handler
    std::unique_ptr<connected_explorers_utils::ConnWeightHandler> conn_weight_handler_;

// data ---
private:
    std::unordered_map<int, geometry_msgs::msg::Pose> robots_poses_guess_; 

// mutex ---
private:
    std::mutex laplacian_row_mutex_;
    std::mutex weights_row_mutex_;
    std::mutex poses_mutex_;

/*******************************************************************************
* Class constructor
*******************************************************************************/
public:
    LaplacianMatrixEstimator() : Node("laplacian_estimator")
    {
        std::string node_param_name;

        node_param_name = "is_3d_mode";
        this->declare_parameter(node_param_name, rclcpp::PARAMETER_BOOL);
        rclcpp::Parameter is_3d_mode_param = this->get_parameter(node_param_name);

        if (is_3d_mode_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            RCLCPP_FATAL(
                this->get_logger(), 
                "Parameter '%s' is required but was not provided! Crashing node.", node_param_name.c_str()
            );
            throw std::runtime_error("Missing required parameter: " + node_param_name);
        }
        problem_dimensions_ = is_3d_mode_param.as_bool() ? 3 : 2;

        node_param_name = "number_of_robots";
        this->declare_parameter<int>(node_param_name, 1);
        number_of_robots_ = this->get_parameter(node_param_name).as_int();
        
        node_param_name = "robot_index";
        this->declare_parameter<int>(node_param_name, 1);
        robot_index_ = this->get_parameter(node_param_name).as_int();

        node_param_name = "los_alpha";
        this->declare_parameter<double>(node_param_name, LOS_ALPHA);
        los_alpha_ = this->get_parameter(node_param_name).as_double();

        node_param_name = "los_beta";
        this->declare_parameter<double>(node_param_name, LOS_BETA);
        los_beta_ = this->get_parameter(node_param_name).as_double();

        node_param_name = "distance_alpha";
        this->declare_parameter<double>(node_param_name, DISTANCE_ALPHA);
        distance_alpha_ = this->get_parameter(node_param_name).as_double();

        node_param_name = "distance_beta";
        this->declare_parameter<double>(node_param_name, DISTANCE_BETA);
        distance_beta_ = this->get_parameter(node_param_name).as_double();

        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&LaplacianMatrixEstimator::init, this)
        );

        publish_timer_ = this->create_wall_timer(
            100ms, std::bind(&LaplacianMatrixEstimator::timerPublishCallback, this)
        );
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init() {
        init_timer_->cancel();
        int qos = static_cast<int>(QOS_STD_PROFILE);

        laplacian_matrix_handler_ = std::make_unique<connected_explorers_utils::LaplacianMatrixHandler>(
            number_of_robots_,
            problem_dimensions_
        );

        conn_weight_handler_ = std::make_unique<connected_explorers_utils::ConnWeightHandler>(
            distance_alpha_, distance_beta_, los_alpha_, los_beta_
        );

        // Branching Map Handlers based on Dimensions
        if (problem_dimensions_ == 3) {
            map_handler_3d_ = std::make_unique<connected_explorers_utils::MapHandler3d>(
                MAP_RESOLUTION,
                this->get_logger()
            );

            auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
            point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                POINT_CLOUD_TOPIC,
                qos_transient,
                std::bind(&LaplacianMatrixEstimator::pointCloudCallback, this, std::placeholders::_1)
            );
        } else {
            // Safe node pointer creation using custom deleter to avoid shared_ptr double free when passing "this"
            std::shared_ptr<rclcpp::Node> node_ptr(this, [](rclcpp::Node*){});
            
            map_handler_2d_ = std::make_shared<connected_explorers_utils::MapHandler>(
                node_ptr,
                MAP_TOPIC,
                qos
            );
            map_handler_2d_->InitMapSubscriber();
        }

        // Publishers
        laplacian_matrix_guess_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "robot" + std::to_string(robot_index_) + "/" + LAPLACIAN_GUESS_TOPIC, qos);
        fiedler_guess_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "robot" + std::to_string(robot_index_) + "/" + FIEDLER_GUESS_TOPIC, qos);
        fiedler_gradient_guess_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "robot" + std::to_string(robot_index_) + "/" + FIEDLER_GRADIENT_GUESS_TOPIC, qos);

        for (int i = 1; i <= number_of_robots_; ++i) {
            std::string topic_name = "robot" + std::to_string(robot_index_) + "/robot" + std::to_string(i) + "/position";
            guess_position_publishers_[i] = this->create_publisher<geometry_msgs::msg::Pose>(topic_name, qos);
        }

        // Subscribers
        neigh_conn_subscriber_ = this->create_subscription<connected_explorers_interfaces::msg::SwarmTopology>(
            "robot" + std::to_string(robot_index_) + "/" + NEIGH_ADJ_TOPIC, 
            qos, std::bind(&LaplacianMatrixEstimator::neighConnWeightCallback, this, std::placeholders::_1));
            
        neigh_pose_subscriber_ = this->create_subscription<connected_explorers_interfaces::msg::RobotPose>(
            "robot" + std::to_string(robot_index_) + "/" + NEIGH_POSE_TOPIC, 
            qos, std::bind(&LaplacianMatrixEstimator::neighPoseCallback, this, std::placeholders::_1));
            
        own_conn_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "robot" + std::to_string(robot_index_) + "/" + CONN_WEIGHT_TOPIC, 
            qos, std::bind(&LaplacianMatrixEstimator::ownConnCallback, this, std::placeholders::_1));
            
        own_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot" + std::to_string(robot_index_) + "/" + POSE_TOPIC, 
            qos, std::bind(&LaplacianMatrixEstimator::ownPoseCallback, this, std::placeholders::_1));
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (map_handler_3d_) {
            map_handler_3d_->BuildInitialKdTree(msg);
        }
    }

    void timerPublishCallback() {
        updateGradientVector();

        std_msgs::msg::Float64MultiArray laplacian_msg = laplacian_matrix_handler_->GetLaplacianMsg();
        std_msgs::msg::Float32 fiedler_msg;
        fiedler_msg.data = laplacian_matrix_handler_->GetFiedlerValue();
        
        std_msgs::msg::Float64MultiArray fiedler_gradient_msg;
        fiedler_gradient_msg.data = laplacian_matrix_handler_->GetGradient();
        int vec_size = fiedler_gradient_msg.data.size();

        if (vec_size > 0) {
            std_msgs::msg::MultiArrayDimension dim_rows;
            dim_rows.label = "rows";
            dim_rows.size = vec_size;
            dim_rows.stride = vec_size;
            fiedler_gradient_msg.layout.dim.push_back(dim_rows);

            std_msgs::msg::MultiArrayDimension dim_cols;
            dim_cols.label = "cols";
            dim_cols.size = 1;
            dim_cols.stride = 1;
            fiedler_gradient_msg.layout.dim.push_back(dim_cols);
        }

        laplacian_matrix_guess_publisher_->publish(laplacian_msg);
        fiedler_guess_publisher_->publish(fiedler_msg);
        fiedler_gradient_guess_publisher_->publish(fiedler_gradient_msg);

        {
            std::lock_guard<std::mutex> lock(poses_mutex_);
            for (const auto& [id, pose] : robots_poses_guess_) {
                auto it = guess_position_publishers_.find(id);
                if (it != guess_position_publishers_.end()) {
                    it->second->publish(pose);
                }
            }
        }
    }

    // Helper: Exact score calculation routed based on dimension
    double CalculateScore(const std::array<double, 3>& pos1, const std::array<double, 3>& pos2) {
        double z1 = (problem_dimensions_ == 3) ? pos1[2] : 0.0;
        double z2 = (problem_dimensions_ == 3) ? pos2[2] : 0.0;

        double obs_dist = 0.0;

        // Dynamic routing
        if (problem_dimensions_ == 3) {
            if (map_handler_3d_ && map_handler_3d_->IsKdTreeReady()) {
                auto coord1 = map_handler_3d_->TransformCoordInVoxel(pos1[0], pos1[1], z1);
                auto coord2 = map_handler_3d_->TransformCoordInVoxel(pos2[0], pos2[1], z2);
                obs_dist = map_handler_3d_->GetMinLineDistanceBetween2Points(coord1, coord2);
            } else {
                obs_dist = 999.0; // Assume free space if map isn't loaded yet
            }
        } else {
            if (map_handler_2d_) {
                // Ensure the map data actually exists before pulling line
                auto grid_line = map_handler_2d_->GetLineBetweenPoints(pos1[0], pos1[1], pos2[0], pos2[1]);
                if (!grid_line.empty()) {
                    obs_dist = map_handler_2d_->GetLineMinDistToObstacle(grid_line).min_dist;
                } else {
                    obs_dist = 999.0;
                }
            } else {
                obs_dist = 999.0;
            }
        }
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = pos1[0]; p1.y = pos1[1]; p1.z = z1;
        p2.x = pos2[0]; p2.y = pos2[1]; p2.z = z2;
        
        double geo_dist = conn_weight_handler_->CalculateDistanceBetweenPoints(p1, p2);

        double dist_score = conn_weight_handler_->CalculateDistanceScore(geo_dist);
        double los_score = conn_weight_handler_->CalculateLoSScore(obs_dist);

        return dist_score * los_score;
    }

    void ComputeStepGradient(
        std::array<double, 3> p_move,
        const std::array<double, 3>& p_fixed,
        double base_weight,
        double eps,
        double& dx, double& dy, double& dz
    ) {
        p_move[0] += eps;
        dx = CalculateScore(p_move, p_fixed) - base_weight; 
        p_move[0] -= eps;

        p_move[1] += eps;
        dy = CalculateScore(p_move, p_fixed) - base_weight;
        p_move[1] -= eps;

        if (problem_dimensions_ == 3) {
            p_move[2] += eps;
            dz = CalculateScore(p_move, p_fixed) - base_weight;
        } else {
            dz = 0.0;
        }
    }

    void updateGradientVector() {
        std::unordered_map<int, std::array<double, 3>> local_poses;
        {
            std::lock_guard<std::mutex> lock(poses_mutex_);
            for (const auto& [id, pose] : robots_poses_guess_) {
                local_poses[id] = {pose.position.x, pose.position.y, pose.position.z};
            }
        }

        for (const auto& [id_i, pose_i] : local_poses) {
            int matrix_idx_i = id_i - 1;

            for (const auto& [id_j, pose_j] : local_poses) {
                if (id_i == id_j) continue;
                int matrix_idx_j = id_j - 1;

                double base_weight = CalculateScore(pose_i, pose_j);

                double dx = 0.0, dy = 0.0, dz = 0.0;
                ComputeStepGradient(pose_i, pose_j, base_weight, MAP_RESOLUTION, dx, dy, dz);

                Eigen::RowVectorXd total_grad(problem_dimensions_);
                if (problem_dimensions_ == 3) {
                    total_grad << dx, dy, dz;
                } else {
                    total_grad << dx, dy;
                }

                {
                    std::lock_guard<std::mutex> lap_lock(laplacian_row_mutex_);
                    laplacian_matrix_handler_->UpdateGradientData(matrix_idx_i, matrix_idx_j, total_grad);
                }
            }
        }

        {
            std::lock_guard<std::mutex> lap_lock(laplacian_row_mutex_);
            laplacian_matrix_handler_->UpdateGradientVector();
        }
    }

    void ownPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        std::lock_guard<std::mutex> lock(poses_mutex_);
        robots_poses_guess_[robot_index_] = *msg;
        if (problem_dimensions_ != 3) {
            robots_poses_guess_[robot_index_].position.z = 0.0;
        }
    }

    void ownConnCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::scoped_lock lock(laplacian_row_mutex_, weights_row_mutex_);
        int my_matrix_index = robot_index_ - 1; 

        for (size_t i = 0; i < msg->data.size(); ++i) {
            if (i == static_cast<size_t>(my_matrix_index)) continue; 
            laplacian_matrix_handler_->UpdateConnWeight(msg->data[i], my_matrix_index, i);
        }
    }

    void neighConnWeightCallback(const connected_explorers_interfaces::msg::SwarmTopology::SharedPtr msg) {
        int id = msg->robot_id;
        float weight = msg->conn_weight;
        std::scoped_lock lock(laplacian_row_mutex_, weights_row_mutex_);

        Eigen::MatrixXd adjacency_matrix = laplacian_matrix_handler_->GetAdjacencyMatrix();

        for (size_t i = 0; i < msg->adjacency_data.size(); ++i) {
            double delta = msg->adjacency_data[i] - adjacency_matrix(i, id - 1); 
            double step = delta * CONN_CORRECTION_STEP * weight;
            double new_value = adjacency_matrix(i, id - 1) + step;
            laplacian_matrix_handler_->UpdateConnWeight(new_value, i, id - 1);
        }
    }

    void neighPoseCallback(const connected_explorers_interfaces::msg::RobotPose::SharedPtr msg) {
        int id = msg->robot_id;
        float weight = msg->conn_weight;
        std::lock_guard<std::mutex> lock(poses_mutex_);

        if (robots_poses_guess_.find(id) == robots_poses_guess_.end()) {
            geometry_msgs::msg::Pose initial_pose = msg->pose;
            if (problem_dimensions_ != 3) {
                initial_pose.position.z = 0.0;
            }
            robots_poses_guess_[id] = initial_pose;
            return;
        }

        double delta_x = msg->pose.position.x - robots_poses_guess_[id].position.x;
        double delta_y = msg->pose.position.y - robots_poses_guess_[id].position.y;
        
        robots_poses_guess_[id].position.x += delta_x * POSE_CORRETCTION_STEP * weight;
        robots_poses_guess_[id].position.y += delta_y * POSE_CORRETCTION_STEP * weight;
        
        if (problem_dimensions_ == 3) {
            double delta_z = msg->pose.position.z - robots_poses_guess_[id].position.z;
            robots_poses_guess_[id].position.z += delta_z * POSE_CORRETCTION_STEP * weight;
        }

        // Pass through orientation from neighbor message
        robots_poses_guess_[id].orientation = msg->pose.orientation;
    }
};

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaplacianMatrixEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}