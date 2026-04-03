#include "std_msgs/msg/color_rgba.hpp"

class ColorClassifier
{
private:
    /* data */
public:
    ColorClassifier(/* args */);
    ~ColorClassifier();




    std_msgs::msg::ColorRGBA GetColorByTask(uint8_t task_id);
    std_msgs::msg::ColorRGBA GetColorByScore(float score);







};

