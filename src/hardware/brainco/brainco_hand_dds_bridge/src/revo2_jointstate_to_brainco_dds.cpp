#include "brainco_hand_dds_bridge/realtime_dds_publisher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

constexpr std::array<const char *, 6> kJointSuffixes = {
  "thumb_proximal_joint",
  "thumb_metacarpal_joint",
  "index_proximal_joint",
  "middle_proximal_joint",
  "ring_proximal_joint",
  "pinky_proximal_joint",
};

constexpr std::array<double, 6> kDefaultUpperLimitsRad = {
  1.0472,
  1.5184,
  1.4661,
  1.4661,
  1.4661,
  1.4661,
};

std::string default_input_topic(const std::string & side)
{
  return "/revo2_" + side + "/revo2_pid_controller/target_joint_states";
}

std::string default_dds_topic(const std::string & side)
{
  return "rt/brainco/" + side + "/cmd";
}

std::vector<double> declare_double_array(
  rclcpp::Node & node,
  const std::string & name,
  const std::array<double, 6> & defaults)
{
  std::vector<double> default_vector(defaults.begin(), defaults.end());
  auto values = node.declare_parameter<std::vector<double>>(name, default_vector);
  if (values.size() != defaults.size()) {
    RCLCPP_WARN(
      node.get_logger(),
      "Parameter %s must contain 6 values; using defaults.",
      name.c_str());
    return default_vector;
  }
  return values;
}

}  // namespace

class Revo2JointStateToBraincoDds : public rclcpp::Node
{
public:
  Revo2JointStateToBraincoDds()
  : Node("revo2_jointstate_to_brainco_dds")
  {
    side_ = this->declare_parameter<std::string>("side", "right");
    const auto network_interface =
      this->declare_parameter<std::string>("network_interface", "");
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", default_input_topic(side_));
    dds_topic_ = this->declare_parameter<std::string>(
      "dds_topic", default_dds_topic(side_));
    command_speed_ = this->declare_parameter<double>("command_speed", 1.0);
    repeat_count_ = this->declare_parameter<int>("repeat_count", 1);
    upper_limits_rad_ = declare_double_array(*this, "upper_limits_rad", kDefaultUpperLimitsRad);

    command_speed_ = std::clamp(command_speed_, 0.0, 1.0);
    repeat_count_ = std::max(1, repeat_count_);

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    publisher_ = std::make_unique<
      brainco_hand_dds_bridge::RealtimeDdsPublisher<unitree_go::msg::dds_::MotorCmds_>>(
      dds_topic_);
    publisher_->msg.cmds().resize(kJointSuffixes.size());
    for (auto & cmd : publisher_->msg.cmds()) {
      cmd.q() = 0.0f;
      cmd.dq() = static_cast<float>(command_speed_);
    }

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        this->on_joint_state(*msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Bridge ready: %s -> %s, side=%s, speed=%.2f",
      input_topic_.c_str(),
      dds_topic_.c_str(),
      side_.c_str(),
      command_speed_);
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() < kJointSuffixes.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring JointState with only %zu positions.", msg.position.size());
      return;
    }

    const auto positions = extract_positions(msg);
    for (size_t i = 0; i < kJointSuffixes.size(); ++i) {
      const double upper = upper_limits_rad_[i] > 0.0 ? upper_limits_rad_[i] : 1.0;
      const double normalized = std::clamp(positions[i] / upper, 0.0, 1.0);
      publisher_->msg.cmds()[i].q() = static_cast<float>(normalized);
      publisher_->msg.cmds()[i].dq() = static_cast<float>(command_speed_);
    }

    for (int i = 0; i < repeat_count_; ++i) {
      publisher_->publish();
    }
  }

  std::array<double, 6> extract_positions(const sensor_msgs::msg::JointState & msg)
  {
    std::array<double, 6> positions{};
    if (msg.name.empty()) {
      std::copy_n(msg.position.begin(), positions.size(), positions.begin());
      return positions;
    }

    std::unordered_map<std::string, double> by_name;
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
      by_name[msg.name[i]] = msg.position[i];
    }

    bool found_all = true;
    for (size_t i = 0; i < kJointSuffixes.size(); ++i) {
      const std::string full_name = side_ + "_" + kJointSuffixes[i];
      auto it = by_name.find(full_name);
      if (it == by_name.end()) {
        it = by_name.find(kJointSuffixes[i]);
      }
      if (it == by_name.end()) {
        found_all = false;
        break;
      }
      positions[i] = it->second;
    }

    if (!found_all) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Joint names did not match expected Revo2 order; falling back to first 6 positions.");
      std::copy_n(msg.position.begin(), positions.size(), positions.begin());
    }
    return positions;
  }

  std::string side_;
  std::string input_topic_;
  std::string dds_topic_;
  double command_speed_{1.0};
  int repeat_count_{1};
  std::vector<double> upper_limits_rad_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::unique_ptr<
    brainco_hand_dds_bridge::RealtimeDdsPublisher<unitree_go::msg::dds_::MotorCmds_>>
    publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Revo2JointStateToBraincoDds>());
  rclcpp::shutdown();
  return 0;
}
