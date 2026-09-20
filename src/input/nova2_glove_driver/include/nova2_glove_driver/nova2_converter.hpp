#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include <SenseGlove/Core/HandPose.hpp>

#include "manus_ros2_msgs/msg/manus_glove.hpp"

namespace nova2_glove_driver
{

struct ChannelMapping
{
  double offset{0.0};
  double scale{1.0};
  double sign{1.0};
  double min{-360.0};
  double max{360.0};
};

struct ConverterOptions
{
  double position_scale{0.001};
  bool publish_raw_nodes{true};
  bool publish_ergonomics{true};
  std::map<std::string, ChannelMapping> mapping;
};

const std::vector<std::string> & ergonomics_field_names();

manus_ros2_msgs::msg::ManusGlove hand_pose_to_manus_glove(
  const SGCore::HandPose & hand_pose,
  int glove_id,
  const std::string & side,
  const ConverterOptions & options);

SGCore::HandPose make_mock_hand_pose(bool right_hand, double t);

}  // namespace nova2_glove_driver
