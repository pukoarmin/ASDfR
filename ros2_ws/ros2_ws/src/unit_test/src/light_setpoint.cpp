/**************************
 * Created 24/02/2023
 * Version 1.0
 * Assignment 1.2.2 Integration of image processing and Jiwy Simulator
**************************/
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp" 

using std::placeholders::_1;
using namespace std::chrono_literals;

class SetPointPublisher : public rclcpp::Node
{
  public:
    SetPointPublisher()
    : Node("light_setpoint")
    {
      //Initial setpoint
      setpoint_.x = 0.0;
      setpoint_.y = 0.0;

      create_topics();  // Creating all the topics
      setpoint_publisher_->publish(setpoint_); // Publishing inital setpoint
    }

  private:
    /**
     * @brief  This function is called whenever new position is received from the jiwy_simulator node. It also publishes the new setpoint
     * 
     */
    
    void position_topic_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received position: [%f, %f]", msg->x, msg->y);
      setpoint_.x = new_coordinate_[1]; // [-0.8 to 0.8] 
      setpoint_.y = new_coordinate_[0]; // [-0.6 to 0.6] 
      setpoint_publisher_->publish(setpoint_);
      RCLCPP_INFO(this->get_logger(), "New setpoint: [%f, %f]", setpoint_.x, setpoint_.y);
    }

    /**
     * @brief Called when light Cog is received from the lighttracker node in /package1/src/light_tracker.cpp
     * 
     */
    void cog_topic_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Light COG received : [%f, %f]", msg->x, msg->y);
        new_coordinate_[0] = -(msg->x/200 - 0.6);             // 0 is x (span direction)
        new_coordinate_[1] = (msg->y/200 - 0.8);              // 1 is y (tilt direction)


    }

    
    void create_topics()
    {
      RCLCPP_INFO(this->get_logger(), "Creating topics...");

      //-subscriber to topic "/position" which is published by jiwy_simulator node
      pos_subscription_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "position", 1, std::bind(&SetPointPublisher::position_topic_callback, this, _1));


      //-publisher to topic "/setpoint" which is subscribed by jiwy_simulator node
      setpoint_publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);


      //-subscription to topic 'cog' from assignment 1.1.4 (Getting center of gravity of the light)
      cog_subscription_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
        "cog",  1, std::bind(&SetPointPublisher::cog_topic_callback, this, _1));
    }

    
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr pos_subscription_;
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr cog_subscription_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr setpoint_publisher_;
    asdfr_interfaces::msg::Point2 setpoint_;
    std::array<float,2> new_coordinate_;  
    int ctr_ = 0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetPointPublisher>());
  rclcpp::shutdown();
  return 0;
}
