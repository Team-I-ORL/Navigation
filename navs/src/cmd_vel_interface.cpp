#include "cmd_vel_interfaces.h"

using std::placeholders::_1;
using std::placeholders::_2;

CmdVelInterface::CmdVelInterface() : Node("cmd_vel_interface")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    service_ = this->create_service<orbiter_bt::srv::CmdVelDist>(
        "cmd_vel_dist", std::bind(&CmdVelInterface::parseCmdVel, this, _1, _2));
}

void CmdVelInterface::parseCmdVel(
    const orbiter_bt::srv::CmdVelDist::Request::SharedPtr request,
    orbiter_bt::srv::CmdVelDist::Response::SharedPtr response)
{
    if (request->fwd_dist != 0.0f && request->spin_dist == 0.0f) {
        float speed = (request->fwd_dist > 0) ? 0.1f : -0.1f;
        float distance = request->fwd_dist;
        float duration = distance / speed;
        RCLCPP_INFO(this->get_logger(), "Moving robot forward by %f m for %f s", distance, duration);

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = speed;
        cmd_vel_msg.angular.z = 0.0f;

        auto start_time = this->now();
        rclcpp::Duration elapsed_time(0, 0);
        rclcpp::Rate loop_rate(10);

        while (elapsed_time.seconds() < duration) {
            publisher_->publish(cmd_vel_msg);
            loop_rate.sleep();
            elapsed_time = this->now() - start_time;
        }

        cmd_vel_msg.linear.x = 0.0f;
        publisher_->publish(cmd_vel_msg);
        response->success = true;
        return;
    } else if (request->spin_dist != 0.0f && request->fwd_dist == 0.0f) {
        float speed = (request->spin_dist > 0) ? 0.1f : -0.1f;
        float angle = request->spin_dist;
        float duration = angle / speed;
        RCLCPP_INFO(this->get_logger(), "Spinning robot by %f rad for %f s", angle, duration);

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0f;
        cmd_vel_msg.angular.z = speed;

        auto start_time = this->now();
        rclcpp::Duration elapsed_time(0, 0);
        rclcpp::Rate loop_rate(10);

        while (elapsed_time.seconds() < duration) {
            publisher_->publish(cmd_vel_msg);
            loop_rate.sleep();
            elapsed_time = this->now() - start_time;
        }

        cmd_vel_msg.angular.z = 0.0f;
        publisher_->publish(cmd_vel_msg);
        response->success = true;
        return;
    } 
    else if (request->fwd_dist != 0.0f && request->spin_dist != 0.0f) {
        float fwd_speed = (request->fwd_dist > 0) ? 0.1f : -0.1f;
        float fwd_distance = request->fwd_dist;
        float fwd_duration = fwd_distance / fwd_speed;

        float spin_angle = request->spin_dist;
        float spin_speed = spin_angle / fwd_duration;

        RCLCPP_INFO(this->get_logger(), "Moving robot forward by %f m for %f s and spinning by %f", fwd_distance, fwd_duration, spin_angle);

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = fwd_speed;
        cmd_vel_msg.angular.z = spin_speed;
        
        auto start_time = this->now();
        rclcpp::Duration elapsed_time(0, 0);
        rclcpp::Rate loop_rate(10);

        while (elapsed_time.seconds() < fwd_duration) {
            publisher_->publish(cmd_vel_msg);
            loop_rate.sleep();
            elapsed_time = this->now() - start_time;
        }

        cmd_vel_msg.angular.z = 0.0f;
        cmd_vel_msg.linear.x = 0.0f;
        publisher_->publish(cmd_vel_msg);
        response->success = true;
    }
        
    else {
        RCLCPP_ERROR(this->get_logger(), "No movement requested.");
        response->success = false;
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelInterface>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}