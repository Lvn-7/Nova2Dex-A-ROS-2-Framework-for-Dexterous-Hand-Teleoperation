#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

struct ReaderOptions
{
  std::string host{"127.0.0.1"};
  int port{15020};
  double rate_hz{60.0};
  bool left{true};
  bool right{true};
  bool start_com{true};
  bool index_influence_others{false};
};

struct ReaderDeviceStatus
{
  std::string sensecom;
  std::string left;
  std::string right;
  std::string left_calibration;
  std::string right_calibration;
  bool sensecom_detected{false};
  bool left_connected{false};
  bool right_connected{false};
};

struct UdpSender
{
  std::atomic_bool running{false};
  std::atomic_uint64_t sent_packets{0};
  std::mutex packet_mutex;
  std::string latest_packet;
  std::thread worker;

  void stop();
  ~UdpSender();
};

void start_sender(UdpSender & sender, const ReaderOptions & options);
std::string format_json(const std::string & json);
ReaderDeviceStatus read_device_status();
