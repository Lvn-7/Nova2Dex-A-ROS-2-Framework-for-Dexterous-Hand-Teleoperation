#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

int main(int argc, char **argv)
{
    if (argc != 3 ||
        (std::string(argv[1]) != "left" && std::string(argv[1]) != "right") ||
        (std::string(argv[2]) != "open" && std::string(argv[2]) != "close"))
    {
        std::cerr << "Usage: " << argv[0] << " <left|right> <open|close>\n";
        return 1;
    }

    const std::string side = argv[1];
    const bool open = std::string(argv[2]) == "open";
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
        command.cmds()[i].q() = open ? 1.0f : 0.0f;
        command.cmds()[i].dq() = 1.0f;
    }

    for (int i = 0; i < 3; ++i)
    {
        publisher->Write(command);
        usleep(100000);
    }
    std::cout << side << " hand: " << (open ? "open" : "close") << std::endl;
    return 0;
}
