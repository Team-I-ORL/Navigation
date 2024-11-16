#include "rclcpp/rclcpp.hpp"
#include "orbiter_bt/srv/cmd_vel_dist.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelInterface : public rclcpp::Node
{
public:
    CmdVelInterface();
    void parseCmdVel(const orbiter_bt::srv::CmdVelDist::Request::SharedPtr request, orbiter_bt::srv::CmdVelDist::Response::SharedPtr response);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Service<orbiter_bt::srv::CmdVelDist>::SharedPtr service_;
};
