#include "illustrator/ColorClassifier.hpp"

#include "connected_explorers_interfaces/msg/robot_task.hpp"

#include <algorithm>
#include <cmath>

#define ALPHA -6.0
#define BETA 0.5
#define DIST_LIM 0.2


using RobotTask = connected_explorers_interfaces::msg::RobotTask;

ColorClassifier::ColorClassifier()
{
}

ColorClassifier::~ColorClassifier()
{
}

std_msgs::msg::ColorRGBA ColorClassifier::GetColorByTask(uint8_t task_id){
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;

    switch (task_id) {
        case RobotTask::TASK:
            color.g = 1.0; break;
        case RobotTask::CONN:
            color.b = 1.0; break;
        default:
            color.r = 1.0; color.g = 1.0; color.b = 0.0; break;
    }
    return color;
}

std_msgs::msg::ColorRGBA ColorClassifier::GetColorByScore(float score){
    float abs_score = std::abs(score);
    float score_threshold = 1.0f / (1.0f + std::exp(ALPHA * (DIST_LIM-BETA)));
    std_msgs::msg::ColorRGBA rgb_result;
    rgb_result.a = 1.0;

    if (abs_score < score_threshold){
        rgb_result.r = 1.0;
        rgb_result.g = 0.0;
        rgb_result.b = 0.0;
    }else{
        float r_val = 1.0f - abs_score;
        rgb_result.r = std::max(0.0f, std::min(1.0f, r_val));
        rgb_result.g = 1.0;
        rgb_result.b = 0.0;
    }

    return rgb_result;
}