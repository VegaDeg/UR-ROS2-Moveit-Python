#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TestPublisher : public rclcpp::Node
{
  public:
    TestPublisher()
    : Node("test_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("target_pos", 10);
      timer_ = this->create_wall_timer(
      5000ms, std::bind(&TestPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto msg = geometry_msgs::msg::Vector3();
      msg.x = 1;
      msg.y = 2;
      msg.z = 3;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f', '%f'", msg.x, msg.y, msg.z);
      publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    float count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
}