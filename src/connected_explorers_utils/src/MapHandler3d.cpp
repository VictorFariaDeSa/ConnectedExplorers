#include "connected_explorers_utils/MapHandler3d.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>



namespace connected_explorers_utils {

MapHandler3d::MapHandler3d(float map_resolution, rclcpp::Logger logger) 
    : map_resolution_(map_resolution), logger_(logger), tree_ready_(false)
{
    kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
}

MapHandler3d::~MapHandler3d() = default;

std::array<int, 3> MapHandler3d::TransformCoordInVoxel(double x, double y, double z) {
    return {
        static_cast<int>(std::round(x / map_resolution_)),
        static_cast<int>(std::round(y / map_resolution_)),
        static_cast<int>(std::round(z / map_resolution_))
    };
}

pcl::PointXYZ MapHandler3d::TransformVoxelInCoord(int grid_x, int grid_y, int grid_z) {
    pcl::PointXYZ searchPoint;
    searchPoint.x = grid_x * map_resolution_;
    searchPoint.y = grid_y * map_resolution_;
    searchPoint.z = grid_z * map_resolution_;
    return searchPoint;
}

std::vector<std::array<int, 3>> MapHandler3d::GetLineBetweenPoints(
    const std::array<int, 3>& coord1,
    const std::array<int, 3>& coord2
){
    std::vector<std::array<int, 3>> points;

    int dx = coord2[0] - coord1[0];
    int dy = coord2[1] - coord1[1];
    int dz = coord2[2] - coord1[2];
    int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});

    points.reserve(steps + 1); // Pre-allocate memory to prevent reallocation

    for (int i = 0; i <= steps; i++) {
        float t = (steps == 0) ? 0.0f : static_cast<float>(i) / steps;
        points.push_back({
            static_cast<int>(std::round(coord1[0] + t * dx)),
            static_cast<int>(std::round(coord1[1] + t * dy)),
            static_cast<int>(std::round(coord1[2] + t * dz))
        });
    }
    return points;
}

bool MapHandler3d::IsKdTreeReady(){
    return kdtree_ != nullptr && tree_ready_;
}

double MapHandler3d::getPointDistanceToObstacle(const std::array<int, 3>& grid_key) {
    auto it = distance_cache_.find(grid_key);
    if (it != distance_cache_.end()) {
        return it->second;
    } 
    
    // 2. Compute if not cached
    if (!IsKdTreeReady()) {
        return 999.0;
    }

    pcl::PointXYZ searchPoint = TransformVoxelInCoord(grid_key[0], grid_key[1], grid_key[2]);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    double dist = 0.0;

    if (kdtree_->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        dist = std::sqrt(pointNKNSquaredDistance[0]);
        distance_cache_[grid_key] = dist; // Store in cache
    }
    
    return dist;
}

double MapHandler3d::GetLineDistanceToObstacle(const std::vector<std::array<int, 3>>& line) {
    double min_dist = std::numeric_limits<double>::max(); // Fixed from float to double
    
    for (const auto& point : line) { // Pass by const ref to avoid copy
        double dist = getPointDistanceToObstacle(point);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}

double MapHandler3d::GetMinLineDistanceBetween2Points(
    const std::array<int, 3>& coord1,
    const std::array<int, 3>& coord2
) {
    std::vector<std::array<int, 3>> conn_line = GetLineBetweenPoints(coord1, coord2);
    return GetLineDistanceToObstacle(conn_line);
}

std::array<double, 3> MapHandler3d::GetMinLineDistanceGradient(
    const std::array<int, 3>& coord1,
    const std::array<int, 3>& coord2
) {
    std::vector<std::array<int, 3>> line = GetLineBetweenPoints(coord1, coord2);
    
    double min_dist = std::numeric_limits<double>::max();
    std::array<int, 3> min_point = coord1;

    // Find the bottleneck point on the line
    for (const auto& point : line) {
        double dist = getPointDistanceToObstacle(point);
        if (dist < min_dist) {
            min_dist = dist;
            min_point = point;
        }
    }

    std::array<double, 3> gradient = {0.0, 0.0, 0.0};

    // If perfectly colliding or tree isn't ready, return zero gradient
    if (min_dist < 1e-5 || !IsKdTreeReady()) {
        return gradient;
    }

    pcl::PointXYZ searchPoint = TransformVoxelInCoord(min_point[0], min_point[1], min_point[2]);
    std::vector<int> pointIdx(1);
    std::vector<float> pointSqrDist(1);

    // Find the exact obstacle coordinate to calculate the directional vector
    if (kdtree_->nearestKSearch(searchPoint, 1, pointIdx, pointSqrDist) > 0) {
        pcl::PointXYZ obstacle_point = kdtree_->getInputCloud()->points[pointIdx[0]];
        
        // 1. Calculate the base spatial gradient at the bottleneck
        double spatial_grad_x = (searchPoint.x - obstacle_point.x) / min_dist;
        double spatial_grad_y = (searchPoint.y - obstacle_point.y) / min_dist;
        double spatial_grad_z = (searchPoint.z - obstacle_point.z) / min_dist;

        // 2. Calculate total length of the line segment
        double dx = coord2[0] - coord1[0];
        double dy = coord2[1] - coord1[1];
        double dz = coord2[2] - coord1[2];
        double total_length = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (total_length > 1e-5) {
            double mdx = min_point[0] - coord1[0];
            double mdy = min_point[1] - coord1[1];
            double mdz = min_point[2] - coord1[2];
            double dist_to_min = std::sqrt(mdx*mdx + mdy*mdy + mdz*mdz);
            
            double u = dist_to_min / total_length;
            
            double weight1 = 1.0 - u;
            gradient[0] = spatial_grad_x * weight1;
            gradient[1] = spatial_grad_y * weight1;
            gradient[2] = spatial_grad_z * weight1;
        } else {
            gradient[0] = spatial_grad_x;
            gradient[1] = spatial_grad_y;
            gradient[2] = spatial_grad_z;
        }
    }

    return gradient;
}

void MapHandler3d::BuildInitialKdTree(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(logger_, "Octomap received! Building 3D distance tree...");

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
        RCLCPP_INFO(logger_, "Tree built successfully with %zu blocks!", cloud->size());
    } else {
        RCLCPP_WARN(logger_, "Octomap loaded, but it has zero occupied blocks.");
    }
}

} // namespace connected_explorers_utils