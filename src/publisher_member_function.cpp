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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using sharedFuture = rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedFuture;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  

  // Creating a Client
    client =
    this->create_client<beginner_tutorials::srv::ChangeString>("change_string");
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
  }

 private:
    std::string Message;
    rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedPtr client;

  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data =
        "Shail Kiritkumar Shah | ENPM808X " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    if (count_%10 == 0) {
      call_service();
    }
    auto steady_clock = rclcpp::Clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(),
         steady_clock, 10000, "Node Running");    
  }

    int call_service() {
    auto request =
        std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
    request->first_string = "Directory";
    request->second_string = "ID";
    RCLCPP_INFO(this->get_logger(), "Calling Service to Modify string");
    auto callbackPtr  =
        std::bind(&MinimalPublisher::response_callback, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }

    void response_callback(sharedFuture success) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "Received String: %s", success.get()->changed_string.c_str());
    Message = success.get()->changed_string.c_str();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
