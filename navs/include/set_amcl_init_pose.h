#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SetAmclInitPose : public rclcpp::Node
{
public:
    SetAmclInitPose();
    void setPose();
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

};
