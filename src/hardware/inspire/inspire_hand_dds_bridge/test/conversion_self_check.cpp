#include "inspire_hand_dds_bridge/conversion.hpp"

#include <cassert>
#include <cmath>

int main()
{
  using inspire_hand_dds_bridge::position_rad_to_command;
  assert(position_rad_to_command(0.0, 1.47) == 1.0);
  assert(position_rad_to_command(1.47, 1.47) == 0.0);
  assert(std::abs(position_rad_to_command(0.735, 1.47) - 0.5) < 1e-12);
  assert(position_rad_to_command(-1.0, 1.47) == 1.0);
  assert(position_rad_to_command(10.0, 1.47) == 0.0);
  return 0;
}

