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

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#define _USE_MATH_DEFINES
 
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SineWavePublisher
{
public:
  SineWavePublisher(ros::NodeHandle nh)
  : nh_(nh), count_(0)
  {
    publisher_ = nh_.advertise<std_msgs::Float64>("topic", 10);
      current_time_ = 0.0;
  }
  void timer_callback()
  {
    auto message = std_msgs::Float64();
    message.data = std::sin(M_PI * current_time_ / 4);
    publisher_.publish(message);
    // std::cout << "Time " << current_time_ << " Value " << message.data << std::endl;
    current_time_ = current_time_ + 0.01;
  }
  private:

  ros::Publisher publisher_;
  ros::NodeHandle nh_;
  size_t count_;
  float current_time_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle nh;
  SineWavePublisher sine_wave_publisher_object(nh);
  ros::Timer timerPublishTemperature = nh.createTimer(ros::Duration(0.01), std::bind(&SineWavePublisher::timer_callback, sine_wave_publisher_object));

  ros::spin();
  return 0;
}
