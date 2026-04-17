/*******************************************************************************
* Description:




*******************************************************************************/



/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"



#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>


// messages
#include "geometry_msgs/msg/pose.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// custom messages
#include "connected_explorers_interfaces/msg/line_clearance_array.hpp"


// custom files
#include "connected_explorers_utils/MultiRobotsPoseHandler.hpp"
#include "connected_explorers_utils/ConnWeightHandler.hpp"
#include "connected_explorers_connections/BitPackHasher.hpp"

/*******************************************************************************
* Defines
*******************************************************************************/

#define QOS_STD_PROFILE 10
#define POSE_TOPIC_NAME "/position"

#define LINE_CLEARANCE_TOPIC_NAME "/line_clearance"

#define MAP_RESOLUTION 0.1

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class DistanceWatcherNode : public rclcpp::Node
{

// parameters ---
private:
    int number_of_robots_;
    std::string robot_name_prefix_;

// publishers ---
private:
    rclcpp::Publisher<connected_explorers_interfaces::msg::LineClearanceArray>::SharedPtr line_clearance_publisher_;

// subscribers ---
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;

// timers ---
private:
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr line_clearance_publisher_timer_;

// helpers ---
private:    
    std::unique_ptr<connected_explorers_utils::MultiRobotsPoseHandler> pose_handler_;
    std::unique_ptr<connected_explorers_utils::ConnWeightHandler> conn_handler_;
    

// data ---
private:
    std::unordered_map<std::array<int, 3>, double, BitpackHasher> distance_cache_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    bool tree_ready_ = false;

        
// mutex ---
private:
    


/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    DistanceWatcherNode() : Node("connection_node")
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
            std::bind(&DistanceWatcherNode::init, this)
        );

        line_clearance_publisher_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&DistanceWatcherNode::PublishLineClearanceMsg, this)
        );
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init() {
        init_timer_->cancel();

        pose_handler_ = std::make_unique<connected_explorers_utils::MultiRobotsPoseHandler>(
            this->shared_from_this(),
            number_of_robots_,
            robot_name_prefix_,
            POSE_TOPIC_NAME,
            QOS_STD_PROFILE
        );

        pose_handler_->InitPoseSubscribers();
        
        
        
        conn_handler_ = std::make_unique<connected_explorers_utils::ConnWeightHandler>(
            1.0f, 6.0f, -6.0f, 0.5f
        );

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/octomap_point_cloud_centers", 
            qos, 
            std::bind(&DistanceWatcherNode::pc_callback, this, std::placeholders::_1)
        );
        
        kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();



        line_clearance_publisher_ = this->create_publisher<connected_explorers_interfaces::msg::LineClearanceArray>(LINE_CLEARANCE_TOPIC_NAME, QOS_STD_PROFILE);


    }


    double CalculateScore(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2) {
        auto coord1 = TransformCoordInVoxel(p1.position.x, p1.position.y, p1.position.z, MAP_RESOLUTION);
        auto coord2 = TransformCoordInVoxel(p2.position.x, p2.position.y, p2.position.z, MAP_RESOLUTION);

        auto line = GetLineBetweenPoints(coord1, coord2);
        double obs_dist = GetLineDistanceToObstacle(line);
        double geo_dist = conn_handler_->CalculateDistanceBetweenPoints(p1.position, p2.position);

        double dist_score = conn_handler_->CalculateDistanceScore(geo_dist);
        double los_score = conn_handler_->CalculateLoSScore(obs_dist);

        return dist_score * los_score;
    }

    void ComputeStepGradient(
        geometry_msgs::msg::Pose p_move, 
        const geometry_msgs::msg::Pose& p_fixed, 
        double base_weight,
        double eps,
        double& dx, double& dy, double& dz
    ) {
        p_move.position.x += eps;
        dx = CalculateScore(p_move, p_fixed) - base_weight;
        p_move.position.x -= eps;

        p_move.position.y += eps;
        dy = CalculateScore(p_move, p_fixed) - base_weight;
        p_move.position.y -= eps;

        p_move.position.z += eps;
        dz = CalculateScore(p_move, p_fixed) - base_weight;
    }

    void PublishLineClearanceMsg() {
        connected_explorers_interfaces::msg::LineClearanceArray batch_msg;
        const auto& robot_poses = pose_handler_->GetRobotsPoses();

        for (int i = 0; i < number_of_robots_ - 1; ++i) {
            for (int j = i + 1; j < number_of_robots_; ++j) {
                
                connected_explorers_interfaces::msg::LineClearance msg;
                msg.robot1_id = i;
                msg.robot2_id = j;

                msg.weight = CalculateScore(robot_poses[i], robot_poses[j]);
                ComputeStepGradient(robot_poses[i], robot_poses[j],msg.weight,MAP_RESOLUTION, msg.dx1, msg.dy1, msg.dz1);
                ComputeStepGradient(robot_poses[j], robot_poses[i],msg.weight,MAP_RESOLUTION, msg.dx2, msg.dy2, msg.dz2);

                batch_msg.clearances.push_back(msg);
            }
        }
        line_clearance_publisher_->publish(batch_msg);
    }


    double getPointDistanceToObstacle(std::array<int, 3> grid_key){
        double dist = 0.0;

        if (distance_cache_.find(grid_key) != distance_cache_.end()) {
            dist = distance_cache_[grid_key];
        } else {
            if (kdtree_ == nullptr || !tree_ready_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                     "KDTree not ready yet! Returning safe default distance.");
                return 999.0; 
            }
            pcl::PointXYZ searchPoint;
            searchPoint.x = grid_key[0] * MAP_RESOLUTION;
            searchPoint.y = grid_key[1] * MAP_RESOLUTION;
            searchPoint.z = grid_key[2] * MAP_RESOLUTION;

            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdtree_->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                dist = std::sqrt(pointNKNSquaredDistance[0]);
                distance_cache_[grid_key] = dist;
            }
        }
        return dist;

    }

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Octomap received! Building 3D distance tree...");

        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                cloud->push_back(pcl::PointXYZ(*iter_x, *iter_y, *iter_z));
            }
        }

        if (!cloud->empty()) {
            kdtree_->setInputCloud(cloud);
            distance_cache_.clear();
            tree_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "Tree built successfully with %zu blocks!", cloud->size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Octomap loaded, but it has zero occupied blocks.");
        }
    }


    std::array<int, 3> TransformCoordInVoxel(double x,double y,double z, double grid_resolution){
        int ix = std::round(x / grid_resolution);
        int iy = std::round(y / grid_resolution);
        int iz = std::round(z / grid_resolution);
        std::array<int, 3> grid_key = {ix, iy, iz};
        return grid_key;
    }



    double GetLineDistanceToObstacle(std::vector<std::array<int, 3>> line){
        double min = std::numeric_limits<float>::max();
        for (std::array<int, 3> point:line){
            double dist = getPointDistanceToObstacle(point);
            if (dist<min){
                min = dist;
            }
        }
        return min;
    }

    std::vector<std::array<int, 3>> GetLineBetweenPoints(
        std::array<int, 3> coord1, 
        std::array<int, 3> coord2
    ) {
        std::vector<std::array<int, 3>> points;

        int dx = coord2[0] - coord1[0], dy = coord2[1] - coord1[1], dz = coord2[2] - coord1[2];
        int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});

        for (int i = 0; i <= steps; i++) {
            float t = (steps == 0) ? 0 : (float)i / steps;
            points.push_back({
                (int)std::round(coord1[0] + t * dx),
                (int)std::round(coord1[1] + t * dy),
                (int)std::round(coord1[2] + t * dz)
            });
        }
        return points;
    }




};


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceWatcherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}