#ifndef MULTI_ROBOTS_POSE_HANDLER_HPP
#define MULTI_ROBOTS_POSE_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <mutex>
#include <vector>
#include <memory>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace connected_explorers_utils {
class MultiRobotsPoseHandler
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    int number_of_robots_;
    std::string robot_name_prefix_;
    std::string pose_topic_name_;
    int qos_profile_;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscriber_list_;
    std::vector<geometry_msgs::msg::PoseStamped> latest_poses_;

    std::mutex poses_mutex_;

public:
    MultiRobotsPoseHandler(
        std::shared_ptr<rclcpp::Node> node,
        int number_of_robots,
        const std::string& robot_name_prefix,
        const std::string& pose_topic_name_,
        int qos_profile
    );
    ~MultiRobotsPoseHandler();

    void InitPoseSubscribers();
    void PoseSubscriberCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int robot_index);
    std::vector<geometry_msgs::msg::PoseStamped> GetRobotsPoses();
    geometry_msgs::msg::PoseStamped GetSingleRobotPose(int index);
};
} // namespace connected_explorers_utils

#endif // MULTI_ROBOTS_POSE_HANDLER_HPP