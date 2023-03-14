/***************************
 * Created 24/02/2023
 * Version 1.0
 * Assignment 1.2.3 Closed Loop Controller
****************************/

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;
using namespace std::chrono_literals;

class ClosedLoopController : public rclcpp::Node
{
    public:
    ClosedLoopController(float dt, float tau)
    :Node("controller")
    {   
      setpoint_.x = 0.0;
      setpoint_.y = 0.0;
      dt_ = dt;
      tau_ = tau;
      create_topics();
      setpoint_timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&ClosedLoopController::setpoint_timer_callback, this));
      setpoint_publisher_->publish(setpoint_);
      
    }

    private:

    void forward_euler(){

       //error between x_light and x_jiwy
      float e_x = (new_coordinate_[1]); // The x and y axes are interchanged because of converting from 3
      float e_y = (new_coordinate_[0]);
      //using forward euler to get the x_set
      setpoint_.x += (1/tau_) * (e_x) * dt_;
      setpoint_.y += (1/tau_) * (e_y) * dt_;

    }

    void position_topic_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg){

      //RCLCPP_INFO(this->get_logger(), "Received position: [%f, %f]", msg->x, msg->y);
      pos_.x = msg->x;
      pos_.y = msg->y;

    }

    void cog_topic_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg){

        RCLCPP_INFO(this->get_logger(), "Light COG received : [%f, %f]", msg->x, msg->y);
        if (msg->x == 0 && msg->y == 0) {
          // No brightest point detected, set new coordinates to centre 
          new_coordinate_[0] = 0;
          new_coordinate_[1] = 0;
        } else {
          new_coordinate_[0] = -(msg->x/200 - 0.3);             // 0 is x (span direction)
          new_coordinate_[1] = (msg->y/200 - 0.4);              // 1 is y (tilt direction)
        }
    }

    void setpoint_timer_callback(){

      forward_euler();
      setpoint_publisher_->publish(setpoint_);
      RCLCPP_INFO(this->get_logger(), "New Setpoint (timer):  [x,y] = [%f,%f]", setpoint_.x, setpoint_.y);

    }
    void create_topics()
    {
      RCLCPP_INFO(this->get_logger(), "Creating topics...");
      //-subscriber to topic position which is published by jiwy_simulator node
      pos_subscription_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "position", 1, std::bind(&ClosedLoopController::position_topic_callback, this, _1));
      //-publisher to topic setpoint which is subscribed by jiwy_simulator node
      setpoint_publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
      //-subscription to light cog from assignment 1.1.4
      cog_subscription_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
        "cog",  1, std::bind(&ClosedLoopController::cog_topic_callback, this, _1));
    }


    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr pos_subscription_;
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr cog_subscription_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr setpoint_publisher_;
    asdfr_interfaces::msg::Point2 setpoint_;
    asdfr_interfaces::msg::Point2 pos_;
    float dt_;
    float tau_;
    std::array<float,2> new_coordinate_;  
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLoopController>(0.1,0.03)); // When tau is 1, the system converges slower. If tau is 0.3 -> the system converges faster 
  rclcpp::shutdown();                                           // But if tau is too small then the system starts to oscillate (0.3 and less) and becomes unstable closer to 0.01
  return 0;
}
