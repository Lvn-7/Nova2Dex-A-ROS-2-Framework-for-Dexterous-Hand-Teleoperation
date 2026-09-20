#pragma once

#include <algorithm>

namespace inspire_hand_dds_bridge
{

inline double position_rad_to_command(double position_rad, double upper_limit_rad)
{
  const double normalized = std::clamp(position_rad / upper_limit_rad, 0.0, 1.0);
  return 1.0 - normalized;
}

}  // namespace inspire_hand_dds_bridge

