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

#include <cstdlib>
#include <iterator>
#include <string>
#include <vector>
#include <memory>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"

using ChangeString = beginner_tutorials::srv::ChangeString;

/**
 * @brief Callback function to process the request
 * 
 * @param request 
 * @param response 
 */
void add(const std::shared_ptr<ChangeString::Request> request,
          std::shared_ptr<ChangeString::Response> response) {
  response->changed_string = request->first_string +" "+ request->second_string+" "+"119340547";
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("modify_string_server");

  rclcpp::Service<ChangeString>::SharedPtr service =
  node->create_service<ChangeString>("change_string", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to change string.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}