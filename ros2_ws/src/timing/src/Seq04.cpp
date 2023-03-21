/***************************
 * Created 15/03/2023
 * Version 1.0
 * Assignment 2.3 Seq node
***************************/
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/custom.hpp" 

using std::placeholders::_1;
using namespace std::chrono_literals;

class Seq04 : public rclcpp::Node
{
  public:
    Seq04()
    : Node("seq04"), count_(0)
    { 
      subscription_ = this->create_subscription<custom_interfaces::msg::Custom>("Reply", 10, std::bind(&Seq04::topic_callback, this, _1));
      publisher_ = this->create_publisher<custom_interfaces::msg::Custom>("Source", 10);
      timer_ = this->create_wall_timer(1ms, std::bind(&Seq04::timer_callback, this));
      msg_.data = "From Seq04";
      msg_.x = 0;
    }

  private:
    void timer_callback()
    {
      auto message = custom_interfaces::msg::Custom();
      message.x = count_++;
      message.data = "From Seq04";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s' '%d", message.data.c_str(), message.x);
      publisher_->publish(message);
    }
    void topic_callback(const custom_interfaces::msg::Custom::SharedPtr msg){

      RCLCPP_INFO(this->get_logger(), "Loop topic received: '%s' '%d", msg->data.c_str(), msg->x);

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::Custom>::SharedPtr publisher_;
    rclcpp::Subscription<custom_interfaces::msg::Custom>::SharedPtr subscription_;
    size_t count_;
    custom_interfaces::msg::Custom msg_;
    //int x_;
    //std::string data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Seq04>());
  rclcpp::shutdown();
  return 0;
}

