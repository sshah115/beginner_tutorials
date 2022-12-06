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
 * @file test.cpp
 * @author Shail Kiritkumar Shah (sshah115@umd.edu)
 * @brief This file contains the definition of all tests being run as
 * part of gtest
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

/**
 * @brief Class definition for testing publisher
 *
 */
class TestPublisher : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr test_node_;
};

TEST_F(TestPublisher, numberTest) {
  test_node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub =
      test_node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto numberPub = test_node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(numberPub));
}
