#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"


class MarkerDesigner
{
private:
    std::string frame_id;

    int line_marker_id;
    int sphere_marker_id;
    
public:
    MarkerDesigner(const std::string& reference_frame);
    ~MarkerDesigner();

    visualization_msgs::msg::Marker GetBaseLineMarkers(
        const rclcpp::Time& stamp,
        const std::string& marker_namespace,
        float line_scale
    );

    visualization_msgs::msg::Marker GetBaseSphereMarkers(
        const rclcpp::Time& stamp,
        const std::string& marker_namespace,
        float diameter
    );

    void AddSphereToMarkerMsg(
        visualization_msgs::msg::Marker& sphere_marker,
        geometry_msgs::msg::Point pose, 
        std_msgs::msg::ColorRGBA color
    );

    void AddLineToMarkerMsg(
        visualization_msgs::msg::Marker& line_marker,
        geometry_msgs::msg::Point pose_1, 
        geometry_msgs::msg::Point pose_2, 
        std_msgs::msg::ColorRGBA color 
    );


private:
};

