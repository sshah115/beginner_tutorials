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

/**
 * @file publisher_member_function.cpp
 * @author Shail Kiritkumar Shah (sshah115@umd.edu)
 * @brief Publisher node 'talker' to display message and also respond to client
 * by requesting server response.
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using sharedFuture =
    rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedFuture;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief Publisher class to publish messages to topic and also handle
 * client request by requesting server response.
 */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Creating a Client
    client = this->create_client<beginner_tutorials::srv::ChangeString>(
        "change_string");
    RCLCPP_DEBUG(this->get_logger(), "Client Generated");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    // Parameter for changing frequency of display.
    auto call_freq = rcl_interfaces::msg::ParameterDescriptor();
    call_freq.description = "Set display frequency.";
    this->declare_parameter("freq", 5.0, call_freq);
    // Fetching value of frequency from parameter server
    auto set_freq = this->get_parameter("freq");
    auto freq = set_freq.get_parameter_value().get<std::float_t>();

    // Making subscriber for Parameter
    // and setting up call back to modify frequency
    mod_param_subscriber_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto paramCallbackPtr =
        std::bind(&MinimalPublisher::param_callback, this, _1);
    mod_paramHandle_ =
        mod_param_subscriber_->add_parameter_callback("freq", paramCallbackPtr);

    // Creating publisher and setting frequency of message display
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto time_frame =
        std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        time_frame, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  std::string Message;
  rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedPtr client;

  std::shared_ptr<rclcpp::ParameterEventHandler> mod_param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> mod_paramHandle_;

  /**
   * @brief Timer callback function which prints the message and at every
   * 10th second (because 10 % 10 == 0) calls service for client request.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data =
        "Shail Kiritkumar Shah | ENPM808X " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    if (count_ % 10 == 0) {
      call_service();
    }
    auto steady_clock = rclcpp::Clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), steady_clock, 10000,
                                 "Node Running");
  }

  /**
   * @brief Service calling function which is called every 10th second for
   * taking client request and prints the first two string of message being
   * displayed.
   * @return int
   */
  int call_service() {
    auto request =
        std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
    request->first_string = "Directory";
    request->second_string = "ID";
    RCLCPP_INFO(this->get_logger(), "Calling Service to Modify string");
    auto callbackPtr =
        std::bind(&MinimalPublisher::response_callback, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }

  /**
   * @brief Fetches changed_string from server as response to client
   * request.
   * @param success
   */
  void response_callback(sharedFuture success) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "Received String: %s",
                success.get()->changed_string.c_str());
    Message = success.get()->changed_string.c_str();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  void param_callback(const rclcpp::Parameter& param) {
    if (param.as_double() == 0.0) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Frequency unchanged because it will result in zero division error");
    } else {
      auto time_frame = std::chrono::milliseconds(
          static_cast<int>((1000 / param.as_double())));
      timer_ = this->create_wall_timer(
          time_frame, std::bind(&MinimalPublisher::timer_callback, this));
    }
  }
};

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
