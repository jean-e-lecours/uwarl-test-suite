#include <chrono>
#include <memory>
#include <functional>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ASMPublisher : public rclcpp::Node
{
  public:
    ASMPublisher()
    : Node("asm_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("asm_pos", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ASMPublisher::asm_callback, this));
    }

  private:
    void asm_callback()
    {
      // ASM function goes here
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class GzSubscriber : public rclcpp::Node
{
  public:
    GzSubscriber()
    : Node("gz_subscriber")
    {
      subscription_ = this->create_subscription<gazebo_msgs::msg::ModelState>(
      "gazebo/model_states", 10, std::bind(&GzSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::spin(std::make_shared<ASMPublisher>());
  rclcpp::shutdown();
  return 0;
}