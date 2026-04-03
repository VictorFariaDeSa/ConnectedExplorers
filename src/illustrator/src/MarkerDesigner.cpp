#include "illustrator/MarkerDesigner.hpp"

using Marker = visualization_msgs::msg::Marker;

MarkerDesigner::MarkerDesigner(const std::string& reference_frame):
frame_id(reference_frame){
    line_marker_id = 0;
    sphere_marker_id = 1;
    
}

MarkerDesigner::~MarkerDesigner()
{

}

Marker MarkerDesigner::GetBaseLineMarkers(
    const rclcpp::Time& stamp,
    const std::string& marker_namespace, 
    float line_scale
){
    visualization_msgs::msg::Marker line_marker;

    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = stamp;
    line_marker.ns = marker_namespace;
    line_marker.id = line_marker_id;
    line_marker.type = Marker::LINE_LIST;
    line_marker.action = Marker::ADD;
    line_marker.scale.x = line_scale;
    line_marker.pose.orientation.w = 1.0;
    return line_marker;
}

Marker MarkerDesigner::GetBaseSphereMarkers(
    const rclcpp::Time& stamp,
    const std::string& marker_namespace,
    float diameter
){
    Marker sphere_marker;

    sphere_marker.header.frame_id = frame_id;
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = marker_namespace;
    sphere_marker_id = sphere_marker_id;
    sphere_marker.type = Marker::SPHERE_LIST;
    sphere_marker.action = Marker::ADD;
    sphere_marker.scale.x = diameter;
    sphere_marker.scale.y = diameter;
    sphere_marker.scale.z = diameter;
    sphere_marker.pose.orientation.w = 1.0;
    return sphere_marker;
}

void MarkerDesigner::AddSphereToMarkerMsg(
    Marker& sphere_marker,
    float x, float y, float z, 
    std_msgs::msg::ColorRGBA color
){
    geometry_msgs::msg::Point p = xyzToRosPoint(x,y,z);
    sphere_marker.points.push_back(p);
    
    sphere_marker.colors.push_back(color);
}

void MarkerDesigner::AddLineToMarkerMsg(
    Marker& line_marker,
    float x_0, float y_0, float z_0, 
    float x_f, float y_f, float z_f, 
    std_msgs::msg::ColorRGBA color 
){
    geometry_msgs::msg::Point p_0 = xyzToRosPoint(x_0,y_0,z_0);
    line_marker.points.push_back(p_0);
    geometry_msgs::msg::Point p_f= xyzToRosPoint(x_f,y_f,z_f);
    line_marker.points.push_back(p_f);

    line_marker.colors.push_back(color);
    line_marker.colors.push_back(color);
}

geometry_msgs::msg::Point MarkerDesigner::xyzToRosPoint(float x, float y, float z){
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

