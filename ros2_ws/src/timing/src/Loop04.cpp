/***************************
 * Created 15/03/2023
 * Version 1.0
 * Assignment 2.3 Loop node
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

class Loop04 : public rclcpp::Node
{
  public:
    Loop04()
    : Node("loop04"),count_(0)
    {
      subscription_ = this->create_subscription<custom_interfaces::msg::Custom>("Source", 10, std::bind(&Loop04::topic_callback, this, _1));
      publisher_ = this->create_publisher<custom_interfaces::msg::Custom>("Reply", 10);
      timer_ = this->create_wall_timer(1ms, std::bind(&Loop04::timer_callback, this));
      switch_ = false;
      msg_.data = "From Seq04";
      msg_.x = 0;
    }

  private:
    void topic_callback(const custom_interfaces::msg::Custom::SharedPtr msg)
    { 
      RCLCPP_INFO(this->get_logger(), "Seq topic received: '%s', '%d'", msg->data.c_str(), msg->x);
      switch_ = true;
      msg_.data = msg->data;
      msg_.x = msg->x;
    }
    void timer_callback()
    {  
      auto message = custom_interfaces::msg::Custom();
      if (switch_){
        message.data = "From Loop04";
        message.x = msg_.x;
        RCLCPP_INFO(this->get_logger(), "Reply :'%s', '%d'", message.data.c_str(), message.x);
        publisher_->publish(message);
        switch_ = false;
        }
      else {
        message.data = "Nothing received yet...";
        message.x = -1;
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
      }

      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_interfaces::msg::Custom>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::Custom>::SharedPtr publisher_;
    bool switch_;
    size_t count_;
    int x_;
    std::string data_;
    custom_interfaces::msg::Custom msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Loop04>());
  rclcpp::shutdown();
  return 0;
}
