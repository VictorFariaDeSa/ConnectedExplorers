
#include <cmath>
#include "geometry_msgs/msg/point.hpp"


namespace connected_explorers_utils{
class ConnWeightHandler
{
private:
    float dist_alpha_;
    float dist_beta_;
    float los_alpha_;
    float los_beta_;

public:
    ConnWeightHandler(float dist_alpha, float dist_beta,float los_alpha, float los_beta);
    ~ConnWeightHandler();
    float CalculateFinalScore(
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2, 
        float min_dist_to_obstacle
    );

    
private:
    
    static float CalculateDistanceBetweenPoints(const geometry_msgs::msg::Point& p1, 
                                               const geometry_msgs::msg::Point& p2);
    static float CalculateGenericScore(float value, float alpha, float beta);
    float CalculateDistanceScore(float distance);
    float CalculateLoSScore(float distance);
};
}
