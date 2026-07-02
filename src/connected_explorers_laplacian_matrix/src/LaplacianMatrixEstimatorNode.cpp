/*******************************************************************************
* Includes
*******************************************************************************/

// REMEMBER TO UPDATE THE GRADIENT DATA

#include "rclcpp/rclcpp.hpp"

// messages
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp" // Added missing include for geometry_msgs::msg::Pose

// custom messages
#include "connected_explorers_interfaces/msg/swarm_topology.hpp"
#include "connected_explorers_interfaces/msg/robot_pose.hpp"

#include "connected_explorers_laplacian_matrix/LaplacianMatrixHandler.hpp"
#include "connected_explorers_utils/MapHandler3d.hpp"
#include "connected_explorers_utils/ConnWeightHandler.hpp"

#include <chrono> // Added for timer durations
#include <mutex>

using namespace std::chrono_literals;

/*******************************************************************************
* Defines
*******************************************************************************/
#define CONN_CORRECTION_STEP 0.8
#define POSE_CORRETCTION_STEP 0.1

#define QOS_STD_PROFILE 10.0

#define LAPLACIAN_GUESS_TOPIC "laplacian_guess"
#define FIEDLER_GRADIENT_GUESS_TOPIC "gradient_guess"
#define FIEDLER_GUESS_TOPIC "fiedler_guess"
#define NEIGH_POSE_TOPIC "internal_pose"
#define NEIGH_ADJ_TOPIC "internal_adjacency"
#define CONN_WEIGHT_TOPIC "know_connections"
#define POSE_TOPIC "position"
#define MAP_TOPIC "map"


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

// timers ---
private:
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_; // Added timer

// helpers ---
private:
    std::unique_ptr<connected_explorers_utils::LaplacianMatrixHandler> laplacian_matrix_handler_;
    std::unique_ptr<connected_explorers_utils::MapHandler3d> map_handler_3d_;
    std::unique_ptr<connected_explorers_utils::ConnWeightHandler> conn_weight_handler_;
// data ---
private:
    std::unordered_map<int, std::array<float, 3>> robots_poses_guess_; //keys == robot_id

// mutex ---
private:
    std::mutex laplacian_row_mutex_;
    std::mutex weights_row_mutex_;
    std::mutex poses_mutex_;

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
        
        node_param_name = "robot_index";
        this->declare_parameter<int>(node_param_name, 0);
        robot_index_ = this->get_parameter(node_param_name).as_int();

       
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&LaplacianMatrixEstimator::init, this)
        );

        // Initialize Timer (Runs at 10Hz / every 100ms - Adjust duration as needed)
        publish_timer_ = this->create_wall_timer(
            100ms, std::bind(&LaplacianMatrixEstimator::timerPublishCallback, this));
    }


/*******************************************************************************
* Class methods
*******************************************************************************/
private:

    void init(){
        init_timer_->cancel();

        int qos = static_cast<int>(QOS_STD_PROFILE);


        laplacian_matrix_handler_ = std::make_unique<connected_explorers_utils::LaplacianMatrixHandler>(
            number_of_robots_,
            3
        );

        conn_weight_handler_ = std::make_unique<connected_explorers_utils::ConnWeightHandler>(
            DISTANCE_ALPHA, DISTANCE_BETA, LOS_ALPHA, LOS_BETA
        );

        map_handler_3d_ = std::make_unique<connected_explorers_utils::MapHandler3d>(
            0.1,
            this->get_logger()
        );

        // Initialize Publishers
        laplacian_matrix_guess_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "robot" + std::to_string(robot_index_) + "/" + LAPLACIAN_GUESS_TOPIC, 
            qos
        );
        fiedler_guess_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "robot" + std::to_string(robot_index_) + "/" + FIEDLER_GUESS_TOPIC, 
            qos
        );

        fiedler_gradient_guess_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "robot" + std::to_string(robot_index_) + "/" + FIEDLER_GRADIENT_GUESS_TOPIC, 
            qos
        );

        for (int i = 1; i <= number_of_robots_; ++i) {
            // Generates topics like: robot1/robot2/position
            std::string topic_name = "robot" + std::to_string(robot_index_) + "/robot" + std::to_string(i) + "/position";
            guess_position_publishers_[i] = this->create_publisher<geometry_msgs::msg::Pose>(topic_name, qos);
        }

        // Initialize Subscribers
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
    // Timer Callback to publish everything at once
    void timerPublishCallback() 
    {
        // 1. Compute gradients based on latest poses
        updateGradientVector();

        // 2. Fetch the calculated values
        std_msgs::msg::Float64MultiArray laplacian_msg;
        std_msgs::msg::Float32 fiedler_msg;
        std_msgs::msg::Float64MultiArray fiedler_gradient_msg;

        laplacian_msg = laplacian_matrix_handler_->GetLaplacianMsg();
        fiedler_msg.data = laplacian_matrix_handler_->GetFiedlerValue();
        
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

        // 3. Publish all messages
        laplacian_matrix_guess_publisher_->publish(laplacian_msg);
        fiedler_guess_publisher_->publish(fiedler_msg);
        fiedler_gradient_guess_publisher_->publish(fiedler_gradient_msg);

        // Publish pose guesses (copy or iterate under lock to avoid races)
        {
            std::lock_guard<std::mutex> lock(poses_mutex_);
            for (const auto& [id, pose] : robots_poses_guess_) {
                // Check if the publisher for this ID exists
                auto it = guess_position_publishers_.find(id);
                if (it != guess_position_publishers_.end()) {
                    geometry_msgs::msg::Pose pos_msg;
                    pos_msg.position.x = pose[0];
                    pos_msg.position.y = pose[1];
                    pos_msg.position.z = pose[2];
                    it->second->publish(pos_msg);
                }
            }
        }
    }

    // reconstruir a matriz laplaciana enviada pelo vizinho
    void ownPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        std::lock_guard<std::mutex> lock(poses_mutex_);
        robots_poses_guess_[robot_index_] = {
            static_cast<float>(msg->position.x),
            static_cast<float>(msg->position.y),
            static_cast<float>(msg->position.z)
        };
    }

    void ownConnCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Ensure consistent locking order when touching laplacian/weights
        std::scoped_lock lock(laplacian_row_mutex_, weights_row_mutex_);

        // robot_index_ is 1-based. We need it to be 0-based for the matrix.
        int my_matrix_index = robot_index_ - 1; 

        for (size_t i = 0; i < msg->data.size(); ++i) {
            // Prevent self-loops: do not assign a weight to yourself
            if (i == static_cast<size_t>(my_matrix_index)) continue; 

            laplacian_matrix_handler_->UpdateConnWeight(msg->data[i], my_matrix_index, i);
        }
    }

    void updateGradientVector() {
        // Use a smaller eps and central difference for more stable gradients
        double eps = 1e-3;

        // Copy poses under lock to avoid holding locks during heavy computation
        std::unordered_map<int, std::array<float, 3>> local_poses;
        {
            std::lock_guard<std::mutex> lock(poses_mutex_);
            local_poses = robots_poses_guess_;
        }

        for (const auto& [id_i, pose_i] : local_poses) {
            int matrix_idx_i = id_i - 1;

            for (const auto& [id_j, pose_j] : local_poses) {
                if (id_i == id_j) continue;

                int matrix_idx_j = id_j - 1;

                Eigen::RowVectorXd grad(3);
                std::array<float, 3> p_move;

                // X central diff
                p_move = pose_i; p_move[0] += eps;
                double f_plus = CalculateScore(p_move, pose_j);
                p_move = pose_i; p_move[0] -= eps;
                double f_minus = CalculateScore(p_move, pose_j);
                grad(0) = (f_plus - f_minus) / (2.0 * eps);

                // Y central diff
                p_move = pose_i; p_move[1] += eps;
                f_plus = CalculateScore(p_move, pose_j);
                p_move = pose_i; p_move[1] -= eps;
                f_minus = CalculateScore(p_move, pose_j);
                grad(1) = (f_plus - f_minus) / (2.0 * eps);

                // Z central diff
                p_move = pose_i; p_move[2] += eps;
                f_plus = CalculateScore(p_move, pose_j);
                p_move = pose_i; p_move[2] -= eps;
                f_minus = CalculateScore(p_move, pose_j);
                grad(2) = (f_plus - f_minus) / (2.0 * eps);

                // Update the derivative dW_ij / dp_i in the handler (protect laplacian handler)
                {
                    std::lock_guard<std::mutex> lap_lock(laplacian_row_mutex_);
                    laplacian_matrix_handler_->UpdateGradientData(matrix_idx_i, matrix_idx_j, grad);
                }
            }
        }

        // Trigger the mathematical multiplication: v^T * (dL/dp) * v
        {
            std::lock_guard<std::mutex> lap_lock(laplacian_row_mutex_);
            laplacian_matrix_handler_->UpdateGradientVector();
        }
    }

    void neighConnWeightCallback(const connected_explorers_interfaces::msg::SwarmTopology::SharedPtr msg) {
        int id = msg->robot_id;
        float weight = msg->conn_weight;
        // Protect reading/updating adjacency/weights with consistent lock order
        std::scoped_lock lock(laplacian_row_mutex_, weights_row_mutex_);

        // Fetch the Adjacency Matrix under lock
        Eigen::MatrixXd adjacency_matrix = laplacian_matrix_handler_->GetAdjacencyMatrix();

        for (size_t i = 0; i < msg->adjacency_data.size(); ++i) {
            // Compare adjacency weights directly
            double delta = msg->adjacency_data[i] - adjacency_matrix(i, id-1); 
            double step = delta * CONN_CORRECTION_STEP * weight;
            double new_value = adjacency_matrix(i, id-1) + step;
            
            // Now passing a valid adjacency weight
            laplacian_matrix_handler_->UpdateConnWeight(new_value, i, id-1);
        }
    }

    // Fixed naming typo here from neightPoseCallback -> neighPoseCallback
    void neighPoseCallback(const connected_explorers_interfaces::msg::RobotPose::SharedPtr msg) {
        int id = msg->robot_id;
        float weight = msg->conn_weight;
        std::lock_guard<std::mutex> lock(poses_mutex_);

        if (robots_poses_guess_.find(id) == robots_poses_guess_.end()) {
            robots_poses_guess_[id] = {0.0f, 0.0f, 0.0f};
        }

        float delta_x = msg->pose.position.x - robots_poses_guess_[id][0];
        float delta_y = msg->pose.position.y - robots_poses_guess_[id][1];
        float delta_z = msg->pose.position.z - robots_poses_guess_[id][2];

        float step_x = delta_x * POSE_CORRETCTION_STEP * weight;
        float step_y = delta_y * POSE_CORRETCTION_STEP * weight;
        float step_z = delta_z * POSE_CORRETCTION_STEP * weight;

        robots_poses_guess_[id][0] += step_x;
        robots_poses_guess_[id][1] += step_y;
        robots_poses_guess_[id][2] += step_z;
    }

    double CalculateScore(const std::array<float, 3>& pos1, const std::array<float, 3>& pos2) {
        // Convert to geometry_msgs::Point for the conn_weight_handler
        geometry_msgs::msg::Point p1, p2;
        p1.x = pos1[0]; p1.y = pos1[1]; p1.z = pos1[2];
        p2.x = pos2[0]; p2.y = pos2[1]; p2.z = pos2[2];

        // 1. Distance Score
        float geo_dist = conn_weight_handler_->CalculateDistanceBetweenPoints(p1, p2);
        float dist_score = conn_weight_handler_->CalculateDistanceScore(geo_dist);

        // 2. Line of Sight Score
        auto coord1 = map_handler_3d_->TransformCoordInVoxel(p1.x, p1.y, p1.z);
        auto coord2 = map_handler_3d_->TransformCoordInVoxel(p2.x, p2.y, p2.z);
        double obs_dist = map_handler_3d_->GetMinLineDistanceBetween2Points(coord1, coord2);
        float los_score = conn_weight_handler_->CalculateLoSScore(obs_dist);

        return dist_score * los_score;
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