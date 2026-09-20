#pragma once

#include <unitree/robot/channel/channel_publisher.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace inspire_hand_dds_bridge
{

template<typename MessageType>
class PublisherBase : public unitree::robot::ChannelPublisher<MessageType>
{
public:
  explicit PublisherBase(const std::string & topic)
  : unitree::robot::ChannelPublisher<MessageType>(topic)
  {
    this->InitChannel();
  }
};

template<typename MessageType>
class RealtimeDdsPublisher
{
public:
  explicit RealtimeDdsPublisher(const std::string & topic)
  : publisher_(std::make_shared<PublisherBase<MessageType>>(topic))
  {
    thread_ = std::thread(&RealtimeDdsPublisher::publishing_loop, this);
  }

  ~RealtimeDdsPublisher()
  {
    keep_running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void publish(const MessageType & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message_ = message;
    pending_ = true;
  }

private:
  void publishing_loop()
  {
    while (keep_running_) {
      MessageType outgoing;
      bool should_publish = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_) {
          outgoing = message_;
          pending_ = false;
          should_publish = true;
        }
      }
      if (should_publish) {
        publisher_->Write(outgoing, 0);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  std::shared_ptr<PublisherBase<MessageType>> publisher_;
  MessageType message_;
  std::atomic_bool keep_running_{true};
  bool pending_{false};
  std::mutex mutex_;
  std::thread thread_;
};

}  // namespace inspire_hand_dds_bridge
