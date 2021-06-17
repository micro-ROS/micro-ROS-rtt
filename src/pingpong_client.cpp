// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/micro-ROS/micro-ROS-rtt.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <cstdio>
#include <sstream>
#include <list>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class PingPongNode : public rclcpp::Node 
{
public:
  PingPongNode(const rclcpp::NodeOptions& opts)
  : Node("pingpong", opts), count_(0), initialized_(false), padding_("")
  {    
    uint32_t padding_size = this->declare_parameter("padding_size", 0);
    padding_ = std::string(padding_size, ' ');
    max_messages_ = this->declare_parameter("max_messages", 0);
    period_ = this->declare_parameter("period", 0.1f);

    publisher_ = this->create_publisher<std_msgs::msg::Header>("uros_ping", 10);
    
    subscription_ = this->create_subscription<std_msgs::msg::Header>(
      "uros_pong",
      10,
      [this](std_msgs::msg::Header::UniquePtr msg) {        
        // this is not the ideal receive time, but should do if we're running with RT prio
        rclcpp::Time received_time = now();        
        rclcpp::Duration elapsed_time = received_time - msg->stamp;
        bool receive_ok = true;

        if(!initialized_) {
          // print the header, this starts our output
          std::cout << "count status elapsed_time(ns) message_time(ns) received_time(ns) frame_time comment" << std::endl;
          initialized_ = true;
        }

        if(outstanding_.empty()) {
          return;
        }

        // parse data from frame_id
        std::string ping;
        uint32_t count = 0;
        uint64_t timestamp = 0;
        std::stringstream str;
        str.str(msg->frame_id);
        str >> ping >> count >> timestamp;
        
        // look for matching outgoing message, skipping if necessary
        auto expected = outstanding_.front();
        uint32_t expected_count = counts_.front();
        while(expected_count < count) {
          print_status(expected_count, false, elapsed_time, rclcpp::Time(), received_time, 0, " message lost");
          outstanding_.pop_front();
          counts_.pop_front();
          // this should not be possible, but lets handle it anyway
          if(outstanding_.empty()) {
            print_status(expected_count, false, elapsed_time, rclcpp::Time(), received_time, 0, " duplicate message");
            return;
          }
          expected = outstanding_.front();
          expected_count = counts_.front();
        }
        outstanding_.pop_front();
        counts_.pop_front();

        // match found, check content                      
        std::stringstream comment;
        if(msg->frame_id != expected->frame_id) {
          comment << " frame_id mismatch (expected=" << expected->frame_id << "!=" << msg->frame_id << ')';
          receive_ok = false;
        }
        if(msg->stamp != expected->stamp) {
          comment << " stamp mismatch (expected=" << rclcpp::Time(expected->stamp).nanoseconds() 
                  << "!=" << rclcpp::Time(msg->stamp).nanoseconds() << ')';
          receive_ok = false;
        }

        print_status(count, receive_ok, elapsed_time, rclcpp::Time(msg->stamp), received_time, timestamp, comment.str());

        if(max_messages_ > 0 && count_ >= max_messages_) {
          rclcpp::shutdown();
        }
      });

    auto timer_callback =
      [this]() -> void {
        auto message = std::make_shared<std_msgs::msg::Header>();
        std::stringstream str;
        rclcpp::Time send_time = now();
        
        // we encode the send time into the stamp as well as into the frame_id as a consistency check
        str << "ping " << this->count_ << ' ' << send_time.nanoseconds() << padding_;
        message->frame_id = str.str();        
        message->stamp = send_time; 
        
        if(initialized_) {
          counts_.push_back(count_++);
          outstanding_.push_back(message);
        }
        
        this->publisher_->publish(*message);        
      };
    auto p = std::chrono::milliseconds(static_cast<int>(std::round(period_*1000.0f)));
    timer_ = this->create_wall_timer(p, timer_callback);
  }
protected:
  void print_status(uint32_t count, bool receive_ok, const rclcpp::Duration& elapsed, const rclcpp::Time& message_time,
    const rclcpp::Time& receive_time, uint64_t frame_time, const std::string& comment) {
      std::cout << count << (receive_ok ? " OK " : " ERR ") 
                << elapsed.nanoseconds() << ' ' << message_time.nanoseconds() << ' ' 
                << receive_time.nanoseconds() << ' ' << frame_time 
                << " \"" << comment << '"' << std::endl; 
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription_;
  uint32_t count_;
  std::list<uint32_t> counts_;
  std::list<std_msgs::msg::Header::SharedPtr> outstanding_;  
  bool initialized_;
  std::string padding_;
  uint32_t max_messages_;
  float period_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingPongNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  
  return 0;
}
