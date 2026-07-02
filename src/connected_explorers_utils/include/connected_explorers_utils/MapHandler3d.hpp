#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "rclcpp/rclcpp.hpp"
#include "BitPackHasher.hpp"


namespace connected_explorers_utils {
class MapHandler3d
{
private:

    float map_resolution_;

    rclcpp::Logger logger_;
    std::unordered_map<std::array<int, 3>, double, BitpackHasher> distance_cache_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    bool tree_ready_;




public:
    MapHandler3d(float map_resolution, rclcpp::Logger logger);
    ~MapHandler3d();

    std::array<int, 3> TransformCoordInVoxel(double x,double y,double z);
    pcl::PointXYZ TransformVoxelInCoord(int grid_x,int grid_y,int grid_z);
    double getPointDistanceToObstacle(const std::array<int, 3>& grid_key);
    double GetLineDistanceToObstacle(const std::vector<std::array<int, 3>>& line);
        std::vector<std::array<int, 3>> GetLineBetweenPoints(
        const std::array<int, 3>& coord1,
        const std::array<int, 3>& coord2
    );
    double GetMinLineDistanceBetween2Points(
        const std::array<int, 3>& coord1,
        const std::array<int, 3>& coord2
    );
    std::array<double, 3> GetMinLineDistanceGradient(
        const std::array<int, 3>& coord1,
        const std::array<int, 3>& coord2
    );
    bool IsKdTreeReady();
    void BuildInitialKdTree(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
}
