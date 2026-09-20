#include "nova2_glove_driver/nova2_converter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <SenseGlove/Core/Quat.hpp>
#include <SenseGlove/Core/Vect3D.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "manus_ros2_msgs/msg/manus_ergonomics.hpp"
#include "manus_ros2_msgs/msg/manus_raw_node.hpp"

namespace nova2_glove_driver
{
namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;

struct FingerSpec
{
  const char * name;
  const char * chain;
  int node_base;
};

constexpr std::array<FingerSpec, 5> kFingers{{
  {"Thumb", "thumb", 1},
  {"Index", "index", 6},
  {"Middle", "middle", 11},
  {"Ring", "ring", 16},
  {"Pinky", "pinky", 21},
}};

constexpr std::array<const char *, 4> kJointTypes{{"mcp", "pip", "dip", "tip"}};

double clamp(double value, double low, double high)
{
  if (low > high) {
    return value;
  }
  return std::min(std::max(value, low), high);
}

double apply_mapping(double value, const ChannelMapping & mapping)
{
  const double mapped = mapping.sign * (value + mapping.offset) * mapping.scale;
  return clamp(mapped, mapping.min, mapping.max);
}

ChannelMapping mapping_for(const ConverterOptions & options, const std::string & field)
{
  const auto it = options.mapping.find(field);
  return it == options.mapping.end() ? ChannelMapping{} : it->second;
}

double angle_y_deg(
  const std::vector<std::vector<SGCore::Kinematics::Vect3D>> & angles,
  std::size_t finger,
  std::size_t joint)
{
  if (finger >= angles.size() || joint >= angles[finger].size()) {
    return 0.0;
  }
  return static_cast<double>(angles[finger][joint].GetY()) * kRadToDeg;
}

double mcp_spread_deg(
  const std::vector<std::vector<SGCore::Kinematics::Vect3D>> & angles,
  std::size_t finger)
{
  if (finger >= angles.size() || angles[finger].empty()) {
    return 0.0;
  }
  // Matches the official senseglove_ros hardware mapping:
  // jointSubIndex 0 = -poseAngles[finger][0].Z.
  return -static_cast<double>(angles[finger][0].GetZ()) * kRadToDeg;
}

geometry_msgs::msg::Pose pose_from(
  const SGCore::Kinematics::Vect3D & position,
  const SGCore::Kinematics::Quat * rotation,
  double position_scale)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = static_cast<double>(position.GetX()) * position_scale;
  pose.position.y = static_cast<double>(position.GetY()) * position_scale;
  pose.position.z = static_cast<double>(position.GetZ()) * position_scale;
  if (rotation) {
    pose.orientation.x = rotation->GetX();
    pose.orientation.y = rotation->GetY();
    pose.orientation.z = rotation->GetZ();
    pose.orientation.w = rotation->GetW();
  } else {
    pose.orientation.w = 1.0;
  }
  return pose;
}

void append_ergonomic(
  manus_ros2_msgs::msg::ManusGlove & msg,
  const std::string & field,
  double raw_degrees,
  const ConverterOptions & options)
{
  manus_ros2_msgs::msg::ManusErgonomics ergo;
  ergo.type = field;
  ergo.value = static_cast<float>(apply_mapping(raw_degrees, mapping_for(options, field)));
  msg.ergonomics.push_back(ergo);
}

}  // namespace

const std::vector<std::string> & ergonomics_field_names()
{
  static const std::vector<std::string> fields = {
    "ThumbMCPStretch", "ThumbMCPSpread", "ThumbPIPStretch", "ThumbDIPStretch",
    "IndexMCPStretch", "IndexSpread", "IndexPIPStretch", "IndexDIPStretch",
    "MiddleMCPStretch", "MiddleSpread", "MiddlePIPStretch", "MiddleDIPStretch",
    "RingMCPStretch", "RingSpread", "RingPIPStretch", "RingDIPStretch",
    "PinkyMCPStretch", "PinkySpread", "PinkyPIPStretch", "PinkyDIPStretch",
  };
  return fields;
}

manus_ros2_msgs::msg::ManusGlove hand_pose_to_manus_glove(
  const SGCore::HandPose & hand_pose,
  int glove_id,
  const std::string & side,
  const ConverterOptions & options)
{
  manus_ros2_msgs::msg::ManusGlove msg;
  msg.glove_id = glove_id;
  msg.side = side;
  msg.raw_sensor_orientation.w = 1.0;

  const auto & positions = hand_pose.GetJointPositions();
  const auto & rotations = hand_pose.GetJointRotations();
  const auto & angles = hand_pose.GetHandAngles();

  if (options.publish_raw_nodes) {
    manus_ros2_msgs::msg::ManusRawNode palm;
    palm.node_id = 0;
    palm.parent_node_id = -1;
    palm.joint_type = "palm";
    palm.chain_type = "palm";
    palm.pose.orientation.w = 1.0;
    msg.raw_nodes.push_back(palm);

    for (std::size_t finger = 0; finger < kFingers.size(); ++finger) {
      if (finger >= positions.size()) {
        continue;
      }
      const int base = kFingers[finger].node_base;
      for (std::size_t joint = 0; joint < positions[finger].size() && joint < kJointTypes.size(); ++joint) {
        const SGCore::Kinematics::Quat * rotation = nullptr;
        if (finger < rotations.size() && joint < rotations[finger].size()) {
          rotation = &rotations[finger][joint];
        }
        manus_ros2_msgs::msg::ManusRawNode node;
        node.node_id = base + static_cast<int>(joint);
        node.parent_node_id = joint == 0 ? 0 : node.node_id - 1;
        node.joint_type = kJointTypes[joint];
        node.chain_type = kFingers[finger].chain;
        node.pose = pose_from(positions[finger][joint], rotation, options.position_scale);
        msg.raw_nodes.push_back(node);
      }
    }
  }
  msg.raw_node_count = static_cast<int>(msg.raw_nodes.size());

  if (options.publish_ergonomics) {
    for (std::size_t finger = 0; finger < kFingers.size(); ++finger) {
      const std::string prefix = kFingers[finger].name;
      append_ergonomic(msg, prefix + "MCPStretch", angle_y_deg(angles, finger, 0), options);
      append_ergonomic(
        msg,
        finger == 0 ? "ThumbMCPSpread" : prefix + "Spread",
        mcp_spread_deg(angles, finger),
        options);
      append_ergonomic(msg, prefix + "PIPStretch", angle_y_deg(angles, finger, 1), options);
      append_ergonomic(msg, prefix + "DIPStretch", angle_y_deg(angles, finger, 2), options);
    }
  }
  msg.ergonomics_count = static_cast<int>(msg.ergonomics.size());
  msg.raw_sensor_count = 0;
  return msg;
}

SGCore::HandPose make_mock_hand_pose(bool right_hand, double t)
{
  using SGCore::Kinematics::Quat;
  using SGCore::Kinematics::Vect3D;
  std::vector<std::vector<Vect3D>> angles(5, std::vector<Vect3D>(3, Vect3D()));
  std::vector<std::vector<Vect3D>> positions(5, std::vector<Vect3D>(4, Vect3D()));
  std::vector<std::vector<Quat>> rotations(5, std::vector<Quat>(4, Quat()));

  const float curl = static_cast<float>(0.65 * (0.5 + 0.5 * std::sin(t)));
  const float spread = static_cast<float>((right_hand ? -0.25 : 0.25) * std::sin(0.5 * t));
  for (std::size_t f = 0; f < 5; ++f) {
    const float finger_scale = f == 0 ? 0.55f : 1.0f;
    angles[f][0] = Vect3D(0.0f, curl * finger_scale, f == 0 ? spread : 0.0f);
    angles[f][1] = Vect3D(0.0f, curl * 0.7f * finger_scale, 0.0f);
    angles[f][2] = Vect3D(0.0f, curl * 0.4f * finger_scale, 0.0f);
    const float x = static_cast<float>(f) * 35.0f;
    for (std::size_t j = 0; j < 4; ++j) {
      positions[f][j] = Vect3D(x, 25.0f * static_cast<float>(j + 1), -35.0f * curl * static_cast<float>(j));
    }
  }
  return SGCore::HandPose(right_hand, positions, rotations, angles);
}

}  // namespace nova2_glove_driver
