#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

namespace
{
void usage(const char *name)
{
    std::cerr << "Usage: " << name
              << " <left|right> <finger_index 0-5> <position 0-1> <velocity>\n"
              << "Finger order: pinky ring middle index thumb_bend thumb_rotate\n";
}
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        usage(argv[0]);
        return 1;
    }

    const std::string side = argv[1];
    const int finger = std::atoi(argv[2]);
    const double position = std::atof(argv[3]);
    const float velocity = std::atof(argv[4]);

    if (side != "left" && side != "right" || finger < 0 || finger > 5 ||
        position < 0.0 || position > 1.0 || velocity < 0.0f || velocity > 1.0f)
    {
        usage(argv[0]);
        return 1;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, "");
    const std::string topic = "rt/inspire/" + side + "/cmd";
    auto publisher = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(
        topic);
    publisher->InitChannel();

    unitree_go::msg::dds_::MotorCmds_ command;
    command.cmds().resize(6);
    for (int i = 0; i < 6; ++i)
    {
        command.cmds()[i].q() = -1.0f;
        command.cmds()[i].dq() = 0.0f;
    }

    command.cmds()[finger].q() = static_cast<float>(position);
    command.cmds()[finger].dq() = static_cast<float>(velocity);
    for (int i = 0; i < 3; ++i)
    {
        publisher->Write(command);
        usleep(100000);
    }

    std::cout << side << " hand, finger " << finger
              << ", position " << position
              << ", velocity " << velocity << " set successfully\n";
    return 0;
}
