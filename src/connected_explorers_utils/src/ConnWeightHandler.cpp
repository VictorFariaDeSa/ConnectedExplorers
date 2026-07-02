#include "connected_explorers_utils/ConnWeightHandler.hpp"


namespace connected_explorers_utils{
ConnWeightHandler::ConnWeightHandler(
    float dist_alpha, 
    float dist_beta,
    float los_alpha, 
    float los_beta
):
dist_alpha_(dist_alpha),
dist_beta_(dist_beta),
los_alpha_(los_alpha),
los_beta_(los_beta)
{}

ConnWeightHandler::~ConnWeightHandler()
{
}

float ConnWeightHandler::CalculateFinalScore(
    const geometry_msgs::msg::Point& p1, 
    const geometry_msgs::msg::Point& p2, 
    float min_dist_to_obstacle
){
    float distance_between_points = CalculateDistanceBetweenPoints(p1,p2);
    float los_score = CalculateLoSScore(min_dist_to_obstacle);
    float dist_score = CalculateDistanceScore(distance_between_points);
    return los_score*dist_score;
}

float ConnWeightHandler::CalculateDistanceBetweenPoints(
    const geometry_msgs::msg::Point& p1, 
    const geometry_msgs::msg::Point& p2
){
    return sqrt(std::pow(p1.x-p2.x, 2)+std::pow(p1.y-p2.y, 2)+std::pow(p1.z-p2.z, 2));
}

float ConnWeightHandler::CalculateGenericScore(
    float distance, 
    float alpha, 
    float beta
){
    return 1.0 / (1.0 + exp(alpha * (distance - beta)));
}

float ConnWeightHandler::CalculateDistanceScore(float distance){
    return CalculateGenericScore(distance, dist_alpha_, dist_beta_);
}

float ConnWeightHandler::CalculateLoSScore(float distance){
    return CalculateGenericScore(distance, los_alpha_, los_beta_);
}

}