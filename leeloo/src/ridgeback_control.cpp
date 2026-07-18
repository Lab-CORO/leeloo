#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std::chrono_literals;

/* Pub all the time a twist (default 0). Sub to /cmd_vel and publish twist with stamped */

class RidgebackTwist : public rclcpp::Node
{
  public:
    RidgebackTwist()
    : Node("ridgeback_twist"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/r100_0597/joy_teleop/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&RidgebackTwist::timer_callback, this));
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&RidgebackTwist::topic_callback, this, std::placeholders::_1));

    }

private:
  void timer_callback()
  {
    this->message.header.stamp = this->get_clock()->now();
    publisher_->publish(this->message);
  }

  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    this->message.twist.linear.x = msg->linear.x;
    this->message.twist.linear.y = msg->linear.y;
    this->message.twist.linear.z = msg->linear.z;

    this->message.twist.angular.x = msg->angular.x;
    this->message.twist.angular.y = msg->angular.y;
    this->message.twist.angular.z = msg->angular.z;

    // RCLCPP_INFO(this->get_logger(), "Twist received");
  }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    geometry_msgs::msg::TwistStamped message;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RidgebackTwist>());
  rclcpp::shutdown();
  return 0;
}