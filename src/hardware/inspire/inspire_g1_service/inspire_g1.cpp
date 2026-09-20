#include "inspire.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <Eigen/Dense>

#include <cstdint>
#include <algorithm>
#include <functional>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace
{

std::string resolveSerialPort(const std::string & requested)
{
  namespace fs = std::filesystem;
  if (requested != "auto")
  {
    if (!fs::exists(requested))
      throw std::runtime_error("Serial port does not exist: " + requested);
    return requested;
  }

  std::vector<std::string> candidates;
  const fs::path by_id("/dev/serial/by-id");
  if (fs::exists(by_id))
  {
    for (const auto & entry : fs::directory_iterator(by_id))
    {
      if (entry.path().filename().string().find("FTDI") != std::string::npos)
        candidates.push_back(entry.path().string());
    }
  }

  if (candidates.empty())
  {
    for (const auto & entry : fs::directory_iterator("/dev"))
    {
      const auto name = entry.path().filename().string();
      if (name.rfind("ttyUSB", 0) == 0)
        candidates.push_back(entry.path().string());
    }
  }

  if (candidates.size() != 1)
  {
    throw std::runtime_error(
      candidates.empty()
        ? "No FTDI or ttyUSB serial port found"
        : "Multiple serial ports found; select one with --serial");
  }
  return candidates.front();
}

}  // namespace

class InspireRunner
{
public:
  InspireRunner()
  {
    // ============================================================
    // Serial / RS-485 configuration
    // ============================================================

    constexpr speed_t BAUD_RATE = B115200;
    const std::string serial_port = resolveSerialPort(param::serial_port);

    // IMPORTANT:
    // Two devices sharing one RS-485 bus must use different IDs.
    // On this setup, device ID 1 is the left hand and ID 2 is the right hand.
    constexpr uint8_t RIGHT_HAND_ID = 2;
    constexpr uint8_t LEFT_HAND_ID = 1;

    // Open the physical serial port only once.
    serial = std::make_shared<SerialPort>(serial_port, BAUD_RATE);
    std::cout << "  Serial port: " << serial_port << std::endl;

    // Both hand objects share the same SerialPort instance.
    // They are distinguished by their device/slave IDs.
    righthand = std::make_shared<inspire::InspireHand>(
        serial,
        RIGHT_HAND_ID);

    lefthand = std::make_shared<inspire::InspireHand>(
        serial,
        LEFT_HAND_ID);

    // Avoid publishing uninitialized Eigen values.
    qcmd.setZero();
    qspeed.setZero();
    qstate.setZero();

    // ============================================================
    // DDS subscription and publisher
    // ============================================================

    rightcmd =
        std::make_shared<
            unitree::robot::SubscriptionBase<
                unitree_go::msg::dds_::MotorCmds_>>(
            "rt/" + param::ns + "/right/cmd");

    rightcmd->msg_.cmds().resize(6);

    leftcmd =
        std::make_shared<
            unitree::robot::SubscriptionBase<
                unitree_go::msg::dds_::MotorCmds_>>(
            "rt/" + param::ns + "/left/cmd");

    leftcmd->msg_.cmds().resize(6);

    handstate =
        std::make_unique<
            unitree::robot::RealTimePublisher<
                unitree_go::msg::dds_::MotorStates_>>(
            "rt/" + param::ns + "/state");

    handstate->msg_.states().resize(12);

    // ============================================================
    // Start periodic control loop
    // 10000 us = 10 ms = 100 Hz
    // ============================================================

    thread =
        std::make_shared<unitree::common::RecurrentThread>(
            10000,
            std::bind(&InspireRunner::run, this));
  }

  void run()
  {
    // ============================================================
    // Receive DDS command and send positions to both hands
    // ============================================================

    // Each hand has an independent DDS command topic. A command for one hand
    // must never cause a write to the other hand.
    if (!rightcmd->isTimeout() && rightcmd->msg_.cmds().size() >= 6)
    {
      for (int i = 0; i < 6; ++i)
      {
        qcmd(i) = rightcmd->msg_.cmds()[i].q();
        qspeed(i) = rightcmd->msg_.cmds()[i].dq();
      }
      righthand->SetPosition(qcmd.block<6, 1>(0, 0));
      righthand->SetVelocity(
          velocityToRaw(qspeed(0)), velocityToRaw(qspeed(1)),
          velocityToRaw(qspeed(2)), velocityToRaw(qspeed(3)),
          velocityToRaw(qspeed(4)), velocityToRaw(qspeed(5)));
    }

    if (!leftcmd->isTimeout() && leftcmd->msg_.cmds().size() >= 6)
    {
      for (int i = 0; i < 6; ++i)
      {
        qcmd(i + 6) = leftcmd->msg_.cmds()[i].q();
        qspeed(i + 6) = leftcmd->msg_.cmds()[i].dq();
      }
      lefthand->SetPosition(qcmd.block<6, 1>(6, 0));
      lefthand->SetVelocity(
          velocityToRaw(qspeed(6)), velocityToRaw(qspeed(7)),
          velocityToRaw(qspeed(8)), velocityToRaw(qspeed(9)),
          velocityToRaw(qspeed(10)), velocityToRaw(qspeed(11)));
    }

    // ============================================================
    // Read hand states
    // ============================================================

    Eigen::Matrix<double, 6, 1> qtemp;
    qtemp.setZero();

    // Read right hand first.
    const int right_state_result = righthand->GetPosition(qtemp);
    if (right_state_result == 0)
    {
      qstate.block<6, 1>(0, 0) = qtemp;
    }
    else
    {
      if (++right_read_failures % 100 == 1)
        std::cerr << "Right hand position read failed, code="
                  << right_state_result << ", failures=" << right_read_failures << std::endl;
      for (int i = 0; i < 6; ++i)
      {
        handstate->msg_.states()[i].lost()++;
      }
    }

    // Read left hand next.
    const int left_state_result = lefthand->GetPosition(qtemp);
    if (left_state_result == 0)
    {
      qstate.block<6, 1>(6, 0) = qtemp;
    }
    else
    {
      if (++left_read_failures % 100 == 1)
        std::cerr << "Left hand position read failed, code="
                  << left_state_result << ", failures=" << left_read_failures << std::endl;
      for (int i = 0; i < 6; ++i)
      {
        handstate->msg_.states()[i + 6].lost()++;
      }
    }

    // ============================================================
    // Publish DDS state
    // ============================================================

    if (handstate->trylock())
    {
      for (int i = 0; i < 12; ++i)
      {
        handstate->msg_.states()[i].q() = qstate(i);
      }

      handstate->unlockAndPublish();
    }
  }

private:
  static int16_t velocityToRaw(double velocity)
  {
    // DDS dq is normalized to [0, 1]; Inspire velocity uses [0, 1000].
    return static_cast<int16_t>(
        std::clamp(velocity, 0.0, 1.0) * 1000.0);
  }

  // Periodic thread
  unitree::common::ThreadPtr thread;

  // One shared RS-485 serial port
  SerialPort::SharedPtr serial;

  // Two devices on the same bus
  std::shared_ptr<inspire::InspireHand> righthand;
  std::shared_ptr<inspire::InspireHand> lefthand;

  // Right hand: indices 0-5
  // Left hand: indices 6-11
  Eigen::Matrix<double, 12, 1> qcmd;
  Eigen::Matrix<double, 12, 1> qspeed;
  Eigen::Matrix<double, 12, 1> qstate;
  std::size_t right_read_failures{0};
  std::size_t left_read_failures{0};

  // DDS
  std::unique_ptr<
      unitree::robot::RealTimePublisher<
          unitree_go::msg::dds_::MotorStates_>>
      handstate;

  std::shared_ptr<
      unitree::robot::SubscriptionBase<
          unitree_go::msg::dds_::MotorCmds_>>
      rightcmd;
  std::shared_ptr<
      unitree::robot::SubscriptionBase<
          unitree_go::msg::dds_::MotorCmds_>>
      leftcmd;
};

int main(int argc, char **argv)
{
  auto vm = param::helper(argc, argv);

  unitree::robot::ChannelFactory::Instance()->Init(
      0,
      param::network);

  std::cout << " --- Unitree Robotics --- " << std::endl;
  std::cout << "  Inspire Hand Controller  " << std::endl;
  std::cout << "  Right hand ID: 2" << std::endl;
  std::cout << "  Left hand ID: 1" << std::endl;

  InspireRunner runner;

  while (true)
  {
    sleep(1);
  }

  return 0;
}
