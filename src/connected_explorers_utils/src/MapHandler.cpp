#include "connected_explorers_utils/MapHandler.hpp"
namespace connected_explorers_utils {
MapHandler::MapHandler(std::shared_ptr<rclcpp::Node> node, std::string map_topic_name, int qos_profile):
node_(node),map_topic_name_(map_topic_name),qos_profile_(qos_profile)
{
}

MapHandler::~MapHandler()
{
}




void MapHandler::InitMapSubscriber()
{
    map_subscriber_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_name_,
            qos_profile_,
            std::bind(&MapHandler::MapSubscriberCallback,this,std::placeholders::_1)
        );

}

void MapHandler::MapSubscriberCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    RCLCPP_INFO(node_->get_logger(), "Map msg received");
    occupancy_grid_ = *msg;
    ComputeMapDistance();
    generate_distances_colormap("/home/victor/projects/ConnectedExplorers/map.png");
}


cv::Point MapHandler::PositionToPixel(float x, float y) {
    float origin_x = occupancy_grid_.info.origin.position.x;
    float origin_y = occupancy_grid_.info.origin.position.y;
    float res = occupancy_grid_.info.resolution;

    int col = static_cast<int>((x - origin_x) / res);
    int row = static_cast<int>((y - origin_y) / res);

    return cv::Point(col, row);
}

geometry_msgs::msg::Point MapHandler::PixelToPosition(int col, int row) {
    geometry_msgs::msg::Point p;
    float origin_x = occupancy_grid_.info.origin.position.x;
    float origin_y = occupancy_grid_.info.origin.position.y;
    float res = occupancy_grid_.info.resolution;

    p.x = origin_x + (static_cast<float>(col) + 0.5f) * res;
    p.y = origin_y + (static_cast<float>(row) + 0.5f) * res;
    p.z = 0.0;

    return p;
}

void MapHandler::ComputeMapDistance(){
    uint32_t width = occupancy_grid_.info.width;
    uint32_t height = occupancy_grid_.info.height;
    float resolution = occupancy_grid_.info.resolution;

    cv::Mat grid(height, width, CV_8S, occupancy_grid_.data.data());

    cv::Mat mask_free;
    cv::compare(grid, 90, mask_free, cv::CMP_LE);
    cv::Mat mask_unknown;
    cv::compare(grid, 0, mask_unknown, cv::CMP_LT);

    cv::bitwise_and(mask_free, ~mask_unknown, mask_free);

    cv::Mat dist_to_obstacle, dist_to_free, mask_obstacle;

    cv::distanceTransform(mask_free, dist_to_obstacle, cv::DIST_L2, 3);

    cv::bitwise_not(mask_free, mask_obstacle);
    cv::distanceTransform(mask_obstacle, dist_to_free, cv::DIST_L2, 3);

    dist_transform = (dist_to_obstacle - dist_to_free) * resolution;

    cv::Mat dx, dy;
    cv::Sobel(dist_transform, dx, CV_32F, 1, 0, 1, 1.0 / (2.0 * resolution));
    cv::Sobel(dist_transform, dy, CV_32F, 0, 1, 1, 1.0 / (2.0 * resolution));

    dist_transform_x_derivative = dx;
    dist_transform_y_derivative = dy;
}

void MapHandler::generate_distances_colormap(const std::string& path) {
    if (dist_transform.empty()) return;

    cv::Mat normalized_dist;
    double min_val, max_val;
    cv::minMaxLoc(dist_transform, &min_val, &max_val);

    dist_transform.convertTo(normalized_dist, CV_8U, 255.0 / (max_val - min_val), 
                                   -min_val * 255.0 / (max_val - min_val));

    cv::Mat color_mapped;
    cv::applyColorMap(normalized_dist, color_mapped, cv::COLORMAP_JET);

    cv::Mat final_image;
    cv::flip(color_mapped, final_image, 0); 

    std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 3};
    cv::imwrite(path, final_image, compression_params);
}


std::vector<cv::Point> MapHandler::GetLineBetweenPoints(float x1, float y1, float x2, float y2) {
    cv::Point p1 = PositionToPixel(x1, y1);
    cv::Point p2 = PositionToPixel(x2, y2);

    
    if (p1.x < 0 || p1.x >= dist_transform.cols || p1.y < 0 || p1.y >= dist_transform.rows ||
        p2.x < 0 || p2.x >= dist_transform.cols || p2.y < 0 || p2.y >= dist_transform.rows) {
        return {};
    }

    std::vector<cv::Point> points;
    cv::LineIterator it(dist_transform, p1, p2, 8); 
    for(int i = 0; i < it.count; i++, ++it) {
        points.push_back(it.pos());
    }
    return points;
}

LineResult MapHandler::GetLineMinDistToObstacle(const std::vector<cv::Point>& grid_line) {
    LineResult result;
    result.min_dist = std::numeric_limits<float>::max();
    result.min_cell = cv::Point(-1, -1);

    if (grid_line.empty()) return result;

    for (const auto& cell : grid_line) {
        float dist = dist_transform.at<float>(cell.y, cell.x);

        if (dist < result.min_dist) {
            result.min_dist = dist;
            result.min_cell = cell;
        }
    }

    return result;
}




}