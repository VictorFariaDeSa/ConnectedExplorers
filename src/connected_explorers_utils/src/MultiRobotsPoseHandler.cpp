#include "connected_explorers_utils/MultiRobotsPoseHandler.hpp"

namespace connected_explorers_utils {
MultiRobotsPoseHandler::MultiRobotsPoseHandler(
    std::shared_ptr<rclcpp::Node> node,
    int number_of_robots,
    const std::string& robot_name_prefix,
    const std::string& pose_topic_name_,
    int qos_profile
):
    node_(node),
    number_of_robots_(number_of_robots),
    robot_name_prefix_(robot_name_prefix),
    pose_topic_name_(pose_topic_name_),
    qos_profile_(qos_profile)
{
    latest_poses_.resize(number_of_robots_);
    pose_subscriber_list_.reserve(number_of_robots_);
}

MultiRobotsPoseHandler::~MultiRobotsPoseHandler()
{
}

void MultiRobotsPoseHandler::InitPoseSubscribers()
{
    for (int i = 0;i<number_of_robots_;i++){
        std::string topic_name = robot_name_prefix_ + std::to_string(i+1) + pose_topic_name_;

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription = 
        node_->create_subscription<geometry_msgs::msg::Pose>(
            topic_name, 
            qos_profile_,
            [this, i](const geometry_msgs::msg::Pose::SharedPtr msg) {
                this->PoseSubscriberCallback(msg, i);
            }
        );

        pose_subscriber_list_.push_back(subscription);
    }
}

void MultiRobotsPoseHandler::PoseSubscriberCallback(const geometry_msgs::msg::Pose::SharedPtr msg, int robot_index){
    std::lock_guard<std::mutex> lock(poses_mutex_);
    latest_poses_[robot_index] = *msg;
}

std::vector<geometry_msgs::msg::Pose> MultiRobotsPoseHandler::GetRobotsPoses()
{
    std::lock_guard<std::mutex> lock(poses_mutex_);
    return latest_poses_;
}


geometry_msgs::msg::Pose MultiRobotsPoseHandler::GetSingleRobotPose(int index)
{
    if (index < 0 || index >= static_cast<int>(latest_poses_.size())) {
        RCLCPP_ERROR(node_->get_logger(), "Index %d out of bounds for GetSingleRobotPose", index);
        return geometry_msgs::msg::Pose();
    }

    std::lock_guard<std::mutex> lock(poses_mutex_);
    return latest_poses_[index];
}
}