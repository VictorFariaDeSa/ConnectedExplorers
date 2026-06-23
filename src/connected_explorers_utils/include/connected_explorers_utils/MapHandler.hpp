#ifndef MAP_HANDLER_HPP
#define MAP_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cmath>
#include <opencv2/opencv.hpp>



struct LineResult {
    float min_dist;
    cv::Point min_cell;
};

namespace connected_explorers_utils {
class MapHandler
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string map_topic_name_;
    int qos_profile_;


    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    cv::Mat dist_transform;
    cv::Mat dist_transform_x_derivative;
    cv::Mat dist_transform_y_derivative;




public:
    MapHandler(
        std::shared_ptr<rclcpp::Node> node,
        std::string map_topic_name,
        int qos_profile
    );
    ~MapHandler();

    void InitMapSubscriber();
    void MapSubscriberCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void ComputeMapDistance();
    void generate_distances_colormap(const std::string& path);
    cv::Point PositionToPixel(float x, float y);
    geometry_msgs::msg::Point PixelToPosition(int col, int row); 
    std::vector<cv::Point> GetLineBetweenPoints(float x1, float y1, float x2, float y2);
    LineResult GetLineMinDistToObstacle(const std::vector<cv::Point>& grid_line);
};
}

#endif //MAP_HANDLER_HPP