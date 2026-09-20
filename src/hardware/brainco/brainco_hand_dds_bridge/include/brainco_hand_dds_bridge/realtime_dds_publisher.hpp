#pragma once

#include <unitree/robot/channel/channel_publisher.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

namespace brainco_hand_dds_bridge
{

template <typename MessageType>
class PublisherBase : public unitree::robot::ChannelPublisher<MessageType>
{
public:
  explicit PublisherBase(const std::string & topic_name)
  : unitree::robot::ChannelPublisher<MessageType>(topic_name)
  {
    this->InitChannel();
  }
};

template <typename MessageType>
class RealtimeDdsPublisher
{
public:
  explicit RealtimeDdsPublisher(const std::string & topic)
  : publisher_(std::make_shared<PublisherBase<MessageType>>(topic)),
    keep_running_(true),
    turn_(LoopState::Realtime)
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

  MessageType msg;

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    turn_ = LoopState::NonRealtime;
  }

private:
  enum class LoopState { Realtime, NonRealtime };

  void publishing_loop()
  {
    while (keep_running_) {
      MessageType outgoing;
      bool should_publish = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (turn_ == LoopState::NonRealtime) {
          outgoing = msg;
          turn_ = LoopState::Realtime;
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
  std::atomic_bool keep_running_;
  std::atomic<LoopState> turn_;
  std::mutex mutex_;
  std::thread thread_;
};

}  // namespace brainco_hand_dds_bridge
