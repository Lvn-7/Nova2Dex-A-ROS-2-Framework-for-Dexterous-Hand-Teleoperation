#include "inspire_hand_dds_bridge/conversion.hpp"
#include "inspire_hand_dds_bridge/realtime_dds_publisher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

constexpr std::array<const char *, 6> kJointNames = {
  "pinky_proximal_joint",
  "ring_proximal_joint",
  "middle_proximal_joint",
  "index_proximal_joint",
  "thumb_proximal_pitch_joint",
  "thumb_proximal_yaw_joint",
};

constexpr std::array<double, 6> kUpperLimitsRad = {
  1.47, 1.47, 1.47, 1.47, 0.6, 1.308,
};

std::string default_input_topic(const std::string & side)
{
  return "/inspire_" + side + "/retarget/joint_states";
}

std::string default_dds_topic(const std::string & side)
{
  return "rt/inspire/" + side + "/cmd";
}

}  // namespace

class InspireJointStateToDds : public rclcpp::Node
{
public:
  InspireJointStateToDds()
  : Node("inspire_jointstate_to_dds")
  {
    side_ = this->declare_parameter<std::string>("side", "right");
    if (side_ != "left" && side_ != "right") {
      throw std::invalid_argument("side must be left or right");
    }

    const auto network_interface =
      this->declare_parameter<std::string>("network_interface", "");
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", default_input_topic(side_));
    dds_topic_ = this->declare_parameter<std::string>(
      "dds_topic", default_dds_topic(side_));
    command_speed_ = std::clamp(
      this->declare_parameter<double>("command_speed", 0.5), 0.0, 1.0);

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    publisher_ = std::make_unique<
      inspire_hand_dds_bridge::RealtimeDdsPublisher<unitree_go::msg::dds_::MotorCmds_>>(
      dds_topic_);
    command_.cmds().resize(kJointNames.size());

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        this->on_joint_state(*message);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Inspire DDS bridge ready: %s -> %s, side=%s, speed=%.2f; "
      "waiting for the first valid JointState before publishing",
      input_topic_.c_str(), dds_topic_.c_str(), side_.c_str(), command_speed_);
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState & message)
  {
    std::array<double, 6> positions{};
    if (!extract_positions(message, positions)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejected JointState: expected all 6 named Inspire joints with finite positions.");
      return;
    }

    for (std::size_t i = 0; i < positions.size(); ++i) {
      command_.cmds()[i].q() = static_cast<float>(
        inspire_hand_dds_bridge::position_rad_to_command(
          positions[i], kUpperLimitsRad[i]));
      command_.cmds()[i].dq() = static_cast<float>(command_speed_);
    }
    publisher_->publish(command_);

    if (!published_first_command_) {
      published_first_command_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Published first valid command to %s in order "
        "[pinky, ring, middle, index, thumb_bend, thumb_rotate].",
        dds_topic_.c_str());
    }
  }

  bool extract_positions(
    const sensor_msgs::msg::JointState & message,
    std::array<double, 6> & positions) const
  {
    if (message.name.size() != message.position.size()) {
      return false;
    }

    std::unordered_map<std::string, double> by_name;
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      by_name[message.name[i]] = message.position[i];
    }

    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      auto found = by_name.find(kJointNames[i]);
      if (found == by_name.end() || !std::isfinite(found->second)) {
        return false;
      }
      positions[i] = found->second;
    }
    return true;
  }

  std::string side_;
  std::string input_topic_;
  std::string dds_topic_;
  double command_speed_{0.5};
  bool published_first_command_{false};
  unitree_go::msg::dds_::MotorCmds_ command_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::unique_ptr<
    inspire_hand_dds_bridge::RealtimeDdsPublisher<unitree_go::msg::dds_::MotorCmds_>>
    publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InspireJointStateToDds>());
  rclcpp::shutdown();
  return 0;
}
