#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <SenseGlove/Core/HandPose.hpp>

#include "manus_ros2_msgs/msg/manus_glove.hpp"
#include "nova2_glove_driver/nova2_converter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nova2_glove_driver
{

class Nova2GloveDriverNode : public rclcpp::Node
{
public:
  explicit Nova2GloveDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct HandRuntime
  {
    bool enabled{true};
    bool right_hand{false};
    int glove_id{0};
    std::string side;
    std::string topic;
    rclcpp::Publisher<manus_ros2_msgs::msg::ManusGlove>::SharedPtr publisher;
    rclcpp::Time last_success;
    rclcpp::Time last_reconnect_log;
    int consecutive_valid{0};
    bool connected{false};
  };

  void declare_parameters();
  void load_parameters();
  void timer_callback();
  void poll_hand(HandRuntime & hand, const rclcpp::Time & now);
  bool read_hand_pose(HandRuntime & hand, SGCore::HandPose & pose, const rclcpp::Time & now);
  ChannelMapping load_mapping_for(const std::string & field, const ChannelMapping & defaults);

  ConverterOptions left_converter_options_;
  ConverterOptions right_converter_options_;
  HandRuntime left_;
  HandRuntime right_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool use_mock_data_{false};
  bool start_sensecom_{true};
  int timeout_ms_{200};
  int reconnect_interval_ms_{1000};
  int required_valid_frames_{2};
  rclcpp::Time start_time_;
};

}  // namespace nova2_glove_driver
