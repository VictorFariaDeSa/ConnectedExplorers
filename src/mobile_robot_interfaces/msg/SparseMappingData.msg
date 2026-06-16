# Standard header for timestamp and robot ID
std_msgs/Header header

# The raw laser data
sensor_msgs/LaserScan scan

# The extracted pose (Note: using Pose, not PoseStamped, because the header is above)
geometry_msgs/Pose pose

# The calculated confidence weight (0.0 to 1.0)
float32 alpha
