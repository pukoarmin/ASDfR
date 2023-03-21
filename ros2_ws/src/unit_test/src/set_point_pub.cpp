/***************************
 * Created 24/02/2023
 * Version 1.0
 * Assignment 1.2.1 Unit Test of Jiwy Simulator
***************************/
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;
using namespace std::chrono_literals;


class SetPointPublisher : public rclcpp::Node
{
  public:
    SetPointPublisher()
    : Node("set_point_pub") //Node name set_point_pub
    {
      //Setting an initial setpoint;
      setpoint_.x = 0.5;
      setpoint_.y = 0.4;
      //creating topics
      create_topics();
      // Publishing the initial setpoint
      setpoint_publisher_->publish(setpoint_);
    }

  private:
    // The position from the jiwy-simulator node is received. A new setpoint, which is randomized, is also published in this callback function
    void position_topic_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received position and setpoint: [%f, %f, %f, %f]", msg->x, msg->y, setpoint_.x, setpoint_.y);

      // If the setpoint has been reached within a specified margin, gen a random setpoint and publish it
      if((fabs(msg->x - setpoint_.x) < 0.0001) && (fabs(msg->y - setpoint_.y) < 0.0001)){
        setpoint_.x = ((float)(rand()%17))/10 - 0.8; // [-0.8 to 0.8] with a 0.1 step
        setpoint_.y = ((float)(rand()%13))/10 - 0.6; // [-0.6 to 0.6] with a 0.1 step
        setpoint_publisher_->publish(setpoint_);
        ctr_ = 0;
      } 
      // If the setpoint has not been reached within 300 position updates (3 seconds @100Hz), republish the current setpoint
      // This handles the scenario when the initial setpoint published at construction/or above was missed by the jiwy_simulator
      else if (ctr_ >= 300) {
        setpoint_publisher_->publish(setpoint_);
        ctr_ = 0;
      } 
      else {
        ctr_++;
      }

    }

    /**
     * @brief Create all topics for this node
     * 
     */
    void create_topics()
    {
      RCLCPP_INFO(this->get_logger(), "Creating topics...");
      //-subscriber to topic position which is published by jiwy_simulator node
      pos_subscription_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "position", 1, std::bind(&SetPointPublisher::position_topic_callback, this, _1));
      //-publisher to topic setpoint which is subscribed by jiwy_simulator node
      setpoint_publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
    }


    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr pos_subscription_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr setpoint_publisher_;
    asdfr_interfaces::msg::Point2 setpoint_;
    int ctr_ = 0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetPointPublisher>());
  rclcpp::shutdown();
  return 0;
}

