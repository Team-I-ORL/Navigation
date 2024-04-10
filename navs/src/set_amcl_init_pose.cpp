#include "set_amcl_init_pose.h"

SetAmclInitPose::SetAmclInitPose() : Node("set_amcl_init_pose")
{
  this->declare_parameter("x", 0.0);
  this->declare_parameter("y", 0.0);
  this->declare_parameter("theta", 0.0);
  this->declare_parameter("cov", std::pow(0.5, 2));

  while(publisher_->get_subscription_count() == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for AMCL Initial Pose subscriber");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
}

void SetAmclInitPose::setPose()
{
    RCLCPP_INFO(this->get_logger(), "Setting AMCL Initial Pose");

    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < end)
    {
      auto pose = geometry_msgs::msg::PoseWithCovarianceStamped();
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.pose.position.x = this->get_parameter("x").as_double();
      pose.pose.pose.position.y = this->get_parameter("y").as_double();

      tf2::Quaternion q;
      q.setRPY(0, 0, this->get_parameter("theta").as_double());
      q.normalize();
      pose.pose.pose.orientation = tf2::toMsg(q);

      pose.pose.covariance[0] = this->get_parameter("cov").as_double();
      pose.pose.covariance[7] = this->get_parameter("cov").as_double();
      pose.pose.covariance[35] = std::pow(0.2, 2);
      this->publisher_->publish(pose);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "AMCL Initial Pose set");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetAmclInitPose>();
  node->setPose();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}