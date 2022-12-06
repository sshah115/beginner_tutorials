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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

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
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Client Generated");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL_STREAM(
            rclcpp::get_logger("rclcpp"),
            "Interrupted while waiting for the service. Exiting.");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                         "Service unavailable, waiting for response...");
    }

    // Parameter for changing frequency of display.
    auto call_freq = rcl_interfaces::msg::ParameterDescriptor();
    call_freq.description = "Set display frequency.";
    this->declare_parameter("freq", 5.0, call_freq);
    // Fetching value of frequency from parameter server
    auto set_freq = this->get_parameter("freq");
    auto freq = set_freq.get_parameter_value().get<std::float_t>();
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "Parameter frequency description and setting it to 5.0 hz");

    // Making subscriber for Parameter
    // and setting up call back to modify frequency
    mod_param_subscriber_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "ParameterEventHandler created");
    auto paramCallbackPtr =
        std::bind(&MinimalPublisher::param_callback, this, _1);
    mod_paramHandle_ =
        mod_param_subscriber_->add_parameter_callback("freq", paramCallbackPtr);

    // Creating publisher and setting frequency of message display
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publisher created");
    auto time_frame =
        std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        time_frame, std::bind(&MinimalPublisher::timer_callback, this));

    tf_static_broadcaster_ =std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->make_transforms();
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
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Node Setup Completed");
    auto message = std_msgs::msg::String();
    message.data =
        "Shail Kiritkumar Shah | ENPM808X " + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: " << message.data.c_str());
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Calling Service to Modify string");
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
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Received String: " << success.get()->changed_string.c_str());
    Message = success.get()->changed_string.c_str();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  /**
   * @brief In case of frequency change request through service, this
   * function prints out useful logging messages.
   * @param param
   */
  void param_callback(const rclcpp::Parameter& param) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Update to parameter "
                                               << "\""
                                               << param.get_name().c_str()
                                               << "\":" << param.as_double());
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Base frequency changed, this might affect some features");

    RCLCPP_FATAL_EXPRESSION(
        this->get_logger(), param.as_double() == 0.0,
        "Frequency set to zero and will result in zero division error");
    if (param.as_double() == 0.0) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Frequency unchanged because it will result in zero division error");
    } else {
      auto time_frame = std::chrono::milliseconds(
          static_cast<int>((1000 / param.as_double())));
      timer_ = this->create_wall_timer(
          time_frame, std::bind(&MinimalPublisher::timer_callback, this));
    }
  }
  /**
   * @brief This will initiate the transformation between frames
   * by statically broadcasting the values to the coordinate frames
   */
  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 1;
    t.transform.translation.y = 5;
    t.transform.translation.z = 7;

    t.transform.rotation.x = 4;
    t.transform.rotation.y = 8;
    t.transform.rotation.z = 3;
    t.transform.rotation.w = 6;

    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  // if (argc != 8) {
  //   RCLCPP_INFO(
  //     logger, "Invalid number of parameters\nusage: "
  //     "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
  //     "child_frame_name x y z roll pitch yaw");
  //   return 1;
  // }

  // // As the parent frame of the transform is `world`, it is
  // // necessary to check that the frame name passed is different
  // if (strcmp(argv[1], "world") == 0) {
  //   RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
  //   return 1;
  // }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
