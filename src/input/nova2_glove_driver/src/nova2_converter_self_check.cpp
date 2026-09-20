#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "nova2_glove_driver/nova2_converter.hpp"

int main()
{
  nova2_glove_driver::ConverterOptions options;
  options.mapping["IndexMCPStretch"].sign = 1.0;
  options.mapping["ThumbMCPSpread"].sign = -1.0;

  const auto pose = nova2_glove_driver::make_mock_hand_pose(true, 1.0);
  const auto msg = nova2_glove_driver::hand_pose_to_manus_glove(pose, 1, "right", options);

  assert(msg.side == "right");
  assert(msg.raw_node_count > 0);
  assert(msg.ergonomics_count == 20);

  bool has_index_mcp = false;
  bool has_thumb_spread = false;
  for (const auto & ergo : msg.ergonomics) {
    if (ergo.type == "IndexMCPStretch") {
      has_index_mcp = true;
      assert(std::isfinite(ergo.value));
      assert(ergo.value > 0.0f);
    }
    if (ergo.type == "ThumbMCPSpread") {
      has_thumb_spread = true;
      assert(std::isfinite(ergo.value));
    }
  }
  assert(has_index_mcp);
  assert(has_thumb_spread);

  std::cout << "nova2_converter_self_check passed\n";
  return 0;
}
