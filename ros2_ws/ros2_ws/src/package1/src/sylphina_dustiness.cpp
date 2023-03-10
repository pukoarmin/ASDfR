#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "package1/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "package1/visibility_control.h"

#include "./policy_maps.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  image_tools::ROSCvMatContainer,
  sensor_msgs::msg::Image);

namespace image_tools
{
  class SylphinaDustiness : public rclcpp::Node
  {
    public:
      IMAGE_TOOLS_PUBLIC
      explicit SylphinaDustiness(const rclcpp::NodeOptions & options)
      : Node("isdarkparam", options)
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        // Do not execute if a --help option was provided
        if (help(options.arguments())) {
          exit(0);
        }
        parse_parameters();
        initialize();
      }

    private:
      IMAGE_TOOLS_LOCAL
      void initialize()
      {
        // Set quality of service profile based on command line options.
        auto qos = rclcpp::QoS(
          rclcpp::QoSInitialization(
            history_policy_,
            depth_
        ));

        qos.reliability(reliability_policy_);
        auto callback =
          [this](const image_tools::ROSCvMatContainer & container) {
            process_image(container, show_image_, this->get_logger());
          };

        RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic_.c_str());
        sub_ = create_subscription<image_tools::ROSCvMatContainer>(topic_, qos, callback);

        if (window_name_ == "") {
          window_name_ = sub_->get_topic_name();
        }
      }

      IMAGE_TOOLS_LOCAL
      bool help(const std::vector<std::string> args)
      {
        if (std::find(args.begin(), args.end(), "--help") != args.end() ||
          std::find(args.begin(), args.end(), "-h") != args.end())
        {
          std::stringstream ss;
          ss << "Usage: isdarkparams [-h] [--ros-args [-p param:=value] ...]" << std::endl;
          ss << "Subscribe to an image topic and decide if the image is bright or not. It's advantage over her sister nide us that she has the threshold as a ROS parameter" << std::endl;
          ss << "Example: ros2 run package1 isdark --ros-args -p reliability:=best_effort";
          ss << std::endl << std::endl;
          ss << "Options:" << std::endl;
          ss << "  -h, --help\tDisplay this help message and exit";
          ss << std::endl << std::endl;
          ss << "Parameters:" << std::endl;
          ss << "  reliability\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
          ss << std::endl;
          ss << "  history\tHistory QoS setting. Either 'keep_last' (default) or 'keep_all'.";
          ss << std::endl;
          ss << "\t\tIf 'keep_last', then up to N samples are stored where N is the depth";
          ss << std::endl;
          ss << "  depth\t\tDepth of the publisher queue. Only honored if history QoS is 'keep_last'.";
          ss << " Default value is 10";
          ss << std::endl;
          ss << "  show_image\tShow the image. Either 'true' (default) or 'false'";
          ss << std::endl;
          ss << "  window_name\tName of the display window. Default value is the topic name";
          ss << std::endl;
          ss << "  threshold\tThreshold value for brightness. Default value is 50";
          ss << std::endl;
          std::cout << ss.str();
          return true;
        }
        return false;
      }

      IMAGE_TOOLS_LOCAL
      void parse_parameters()
      {
        // Parse 'reliability' parameter
        rcl_interfaces::msg::ParameterDescriptor reliability_desc;
        reliability_desc.description = "Reliability QoS setting for the image subscription";
        reliability_desc.additional_constraints = "Must be one of: ";
        for (auto entry : name_to_reliability_policy_map) {
          reliability_desc.additional_constraints += entry.first + " ";
        }
        const std::string reliability_param = this->declare_parameter(
          "reliability", "reliable", reliability_desc);
        auto reliability = name_to_reliability_policy_map.find(reliability_param);
        if (reliability == name_to_reliability_policy_map.end()) {
          std::ostringstream oss;
          oss << "Invalid QoS reliability setting '" << reliability_param << "'";
          throw std::runtime_error(oss.str());
        }
        reliability_policy_ = reliability->second;

        // Parse 'history' parameter
        rcl_interfaces::msg::ParameterDescriptor history_desc;
        history_desc.description = "History QoS setting for the image subscription";
        history_desc.additional_constraints = "Must be one of: ";
        for (auto entry : name_to_history_policy_map) {
          history_desc.additional_constraints += entry.first + " ";
        }
        const std::string history_param = this->declare_parameter(
          "history", name_to_history_policy_map.begin()->first, history_desc);
        auto history = name_to_history_policy_map.find(history_param);
        if (history == name_to_history_policy_map.end()) {
          std::ostringstream oss;
          oss << "Invalid QoS history setting '" << history_param << "'";
          throw std::runtime_error(oss.str());
        }
        history_policy_ = history->second;

        // Declare and get remaining parameters
        depth_ = this->declare_parameter("depth", 10);
        show_image_ = this->declare_parameter("show_image", true);
        window_name_ = this->declare_parameter("window_name", "");
        threshold_ = this->declare_parameter("threshold", 50);
      }

      /// Convert the ROS Image message to an OpenCV matrix and display it to the user.
      // \param[in] container The image message to show.
      IMAGE_TOOLS_LOCAL
      void process_image(
        const image_tools::ROSCvMatContainer & container, bool show_image, rclcpp::Logger logger)
      {
        RCLCPP_INFO(logger, "Received image #%s", container.header().frame_id.c_str());

        cv::Mat frame = container.cv_mat();

        if (frame.type() == CV_8UC3 /* rgb8 */) {
          cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);
        } else if (frame.type() == CV_8UC2) {
          container.is_bigendian() ? cv::cvtColor(frame, frame, cv::COLOR_YUV2GRAY_UYVY) :
          cv::cvtColor(frame, frame, cv::COLOR_YUV2GRAY_YUYV);
        }

        if (show_image) {
          // Show the image in a window
          cv::imshow(window_name_, frame);
          // Draw the screen and wait for 1 millisecond.
          cv::waitKey(1);
        }

        // Determine Medium Brightness
        int medium_brightness = 0;

        for (int i = 0; i < frame.rows; i++) {
          for (int j = 0; j < frame.cols; j++){ 
            medium_brightness += frame.at<uchar>(i, j);
          }
        }
        medium_brightness = medium_brightness / (frame.rows * frame.cols);
        if (medium_brightness < threshold_){
          std::cerr << "It is dark"<< std::endl;
        } else {
          std::cerr << "It is light"<< std::endl;
          std::cerr << medium_brightness<< std::endl;
        }
      }

      rclcpp::Subscription<image_tools::ROSCvMatContainer>::SharedPtr sub_;
      size_t depth_ = rmw_qos_profile_default.depth;
      rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
      rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
      bool show_image_ = true;
      std::string topic_ = "image";
      int threshold_ = 50;
      std::string window_name_;
  };

}

RCLCPP_COMPONENTS_REGISTER_NODE(image_tools::SylphinaDustiness)