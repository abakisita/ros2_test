// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#define _USE_MATH_DEFINES
 
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SineWavePublisher : public rclcpp::Node
{
public:
  SineWavePublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("topic", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&SineWavePublisher::timer_callback, this));
      current_time_ = 0.0;
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float64();
    message.data = std::sin(M_PI * current_time_);
    publisher_->publish(message);
    // std::cout << "Time " << current_time_ << " Value " << message.data << std::endl;
    current_time_ = current_time_ + 0.01;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  size_t count_;
  float current_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineWavePublisher>());
  rclcpp::shutdown();
  return 0;
}
