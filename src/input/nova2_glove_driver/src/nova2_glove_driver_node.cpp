#include "nova2_glove_driver/nova2_glove_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/Library.hpp>
#include <SenseGlove/Core/SenseCom.hpp>

namespace nova2_glove_driver
{
namespace
{

rclcpp::Duration duration_ms(int value)
{
  return rclcpp::Duration::from_seconds(static_cast<double>(std::max(0, value)) / 1000.0);
}

bool is_flex_field(const std::string & field)
{
  return field.find("Stretch") != std::string::npos;
}

bool is_thumb_spread_field(const std::string & field)
{
  return field == "ThumbMCPSpread";
}

}  // namespace

Nova2GloveDriverNode::Nova2GloveDriverNode(const rclcpp::NodeOptions & options)
: Node("nova2_glove_driver", options)
{
  declare_parameters();
  load_parameters();
  start_time_ = now();
  left_.last_success = start_time_;
  right_.last_success = start_time_;
  left_.last_reconnect_log = start_time_ - duration_ms(reconnect_interval_ms_);
  right_.last_reconnect_log = left_.last_reconnect_log;

  left_.publisher = create_publisher<manus_ros2_msgs::msg::ManusGlove>(left_.topic, 10);
  right_.publisher = create_publisher<manus_ros2_msgs::msg::ManusGlove>(right_.topic, 10);

  if (start_sensecom_ && !use_mock_data_ && !SGCore::SenseCom::ScanningActive()) {
    if (SGCore::SenseCom::StartupSenseCom()) {
      RCLCPP_INFO(get_logger(), "Started SenseCom; waiting for Nova 2 connection.");
    } else {
      RCLCPP_WARN(get_logger(), "SenseCom is not active and could not be auto-started.");
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "SenseGlove API %s backend=%s, publishing left=%s (%s), right=%s (%s), mock=%s",
    SGCore::Library::Version().c_str(),
    SGCore::Library::BackendVersion().c_str(),
    left_.topic.c_str(),
    left_.enabled ? "enabled" : "disabled",
    right_.topic.c_str(),
    right_.enabled ? "enabled" : "disabled",
    use_mock_data_ ? "true" : "false");

  const double publish_rate_hz = std::max(1.0, get_parameter("publish_rate_hz").as_double());
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz)),
    [this]() { timer_callback(); });
}

void Nova2GloveDriverNode::declare_parameters()
{
  declare_parameter("publish_rate_hz", 60.0);
  declare_parameter("enable_left", true);
  declare_parameter("enable_right", true);
  declare_parameter("left_topic", "/manus_glove_0");
  declare_parameter("right_topic", "/manus_glove_1");
  declare_parameter("position_scale", 0.001);
  declare_parameter("left_flex_sign", 1.0);
  declare_parameter("right_flex_sign", 1.0);
  declare_parameter("left_thumb_spread_sign", 1.0);
  declare_parameter("right_thumb_spread_sign", -1.0);
  declare_parameter("timeout_ms", 200);
  declare_parameter("reconnect_interval_ms", 1000);
  declare_parameter("required_valid_frames", 2);
  declare_parameter("publish_raw_nodes", true);
  declare_parameter("publish_ergonomics", true);
  declare_parameter("use_mock_data", false);
  declare_parameter("start_sensecom", true);

  for (const auto & field : ergonomics_field_names()) {
    declare_parameter("mapping." + field + ".offset", 0.0);
    declare_parameter("mapping." + field + ".scale", 1.0);
    declare_parameter("mapping." + field + ".sign", 1.0);
    declare_parameter("mapping." + field + ".min", -360.0);
    declare_parameter("mapping." + field + ".max", 360.0);
  }
}

void Nova2GloveDriverNode::load_parameters()
{
  timeout_ms_ = static_cast<int>(get_parameter("timeout_ms").as_int());
  reconnect_interval_ms_ = static_cast<int>(get_parameter("reconnect_interval_ms").as_int());
  required_valid_frames_ = std::max(1, static_cast<int>(get_parameter("required_valid_frames").as_int()));
  use_mock_data_ = get_parameter("use_mock_data").as_bool();
  start_sensecom_ = get_parameter("start_sensecom").as_bool();

  left_.enabled = get_parameter("enable_left").as_bool();
  left_.right_hand = false;
  left_.glove_id = 0;
  left_.side = "left";
  left_.topic = get_parameter("left_topic").as_string();

  right_.enabled = get_parameter("enable_right").as_bool();
  right_.right_hand = true;
  right_.glove_id = 1;
  right_.side = "right";
  right_.topic = get_parameter("right_topic").as_string();

  const double position_scale = get_parameter("position_scale").as_double();
  left_converter_options_.position_scale = position_scale;
  right_converter_options_.position_scale = position_scale;
  left_converter_options_.publish_raw_nodes = get_parameter("publish_raw_nodes").as_bool();
  right_converter_options_.publish_raw_nodes = left_converter_options_.publish_raw_nodes;
  left_converter_options_.publish_ergonomics = get_parameter("publish_ergonomics").as_bool();
  right_converter_options_.publish_ergonomics = left_converter_options_.publish_ergonomics;

  const double left_flex_sign = get_parameter("left_flex_sign").as_double();
  const double right_flex_sign = get_parameter("right_flex_sign").as_double();
  const double left_thumb_spread_sign = get_parameter("left_thumb_spread_sign").as_double();
  const double right_thumb_spread_sign = get_parameter("right_thumb_spread_sign").as_double();

  for (const auto & field : ergonomics_field_names()) {
    ChannelMapping left_defaults;
    ChannelMapping right_defaults;
    if (is_flex_field(field)) {
      left_defaults.sign = left_flex_sign;
      right_defaults.sign = right_flex_sign;
    }
    if (is_thumb_spread_field(field)) {
      left_defaults.sign = left_thumb_spread_sign;
      right_defaults.sign = right_thumb_spread_sign;
    }
    left_converter_options_.mapping[field] = load_mapping_for(field, left_defaults);
    right_converter_options_.mapping[field] = load_mapping_for(field, right_defaults);
  }
}

ChannelMapping Nova2GloveDriverNode::load_mapping_for(
  const std::string & field,
  const ChannelMapping & defaults)
{
  ChannelMapping mapping = defaults;
  const std::string prefix = "mapping." + field + ".";
  mapping.offset = get_parameter(prefix + "offset").as_double();
  mapping.scale = get_parameter(prefix + "scale").as_double();
  const double configured_sign = get_parameter(prefix + "sign").as_double();
  mapping.sign *= configured_sign;
  mapping.min = get_parameter(prefix + "min").as_double();
  mapping.max = get_parameter(prefix + "max").as_double();
  return mapping;
}

void Nova2GloveDriverNode::timer_callback()
{
  const auto stamp = now();
  poll_hand(left_, stamp);
  poll_hand(right_, stamp);
}

void Nova2GloveDriverNode::poll_hand(HandRuntime & hand, const rclcpp::Time & stamp)
{
  if (!hand.enabled) {
    return;
  }

  SGCore::HandPose pose;
  if (!read_hand_pose(hand, pose, stamp)) {
    if (hand.connected && (stamp - hand.last_success) > duration_ms(timeout_ms_)) {
      hand.connected = false;
      hand.consecutive_valid = 0;
      RCLCPP_WARN(get_logger(), "Nova 2 %s timed out; stopping ManusGlove publishing.", hand.side.c_str());
    }
    return;
  }

  hand.last_success = stamp;
  hand.consecutive_valid += 1;
  if (!hand.connected && hand.consecutive_valid >= required_valid_frames_) {
    hand.connected = true;
    RCLCPP_INFO(get_logger(), "Nova 2 %s connected; publishing %s.", hand.side.c_str(), hand.topic.c_str());
  }
  if (!hand.connected) {
    return;
  }

  const auto & options = hand.right_hand ? right_converter_options_ : left_converter_options_;
  auto msg = hand_pose_to_manus_glove(pose, hand.glove_id, hand.side, options);
  hand.publisher->publish(msg);
}

bool Nova2GloveDriverNode::read_hand_pose(
  HandRuntime & hand,
  SGCore::HandPose & pose,
  const rclcpp::Time & stamp)
{
  if (use_mock_data_) {
    pose = make_mock_hand_pose(hand.right_hand, (stamp - start_time_).seconds());
    return true;
  }

  if (!SGCore::SenseCom::ScanningActive()) {
    if ((stamp - hand.last_reconnect_log) > duration_ms(reconnect_interval_ms_)) {
      hand.last_reconnect_log = stamp;
      RCLCPP_WARN(get_logger(), "SenseCom is not active; waiting before reading Nova 2 %s.", hand.side.c_str());
      if (start_sensecom_) {
        SGCore::SenseCom::StartupSenseCom();
      }
    }
    return false;
  }

  if (!SGCore::HandLayer::DeviceConnected(hand.right_hand)) {
    if ((stamp - hand.last_reconnect_log) > duration_ms(reconnect_interval_ms_)) {
      hand.last_reconnect_log = stamp;
      RCLCPP_WARN(get_logger(), "No Nova/SenseGlove %s device detected.", hand.side.c_str());
    }
    return false;
  }

  if (!SGCore::HandLayer::GetHandPose(hand.right_hand, pose)) {
    if ((stamp - hand.last_reconnect_log) > duration_ms(reconnect_interval_ms_)) {
      hand.last_reconnect_log = stamp;
      RCLCPP_WARN(get_logger(), "Failed to read Nova 2 %s HandPose.", hand.side.c_str());
    }
    return false;
  }

  return true;
}

}  // namespace nova2_glove_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nova2_glove_driver::Nova2GloveDriverNode>());
  rclcpp::shutdown();
  return 0;
}
