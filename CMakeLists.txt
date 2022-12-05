cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"  
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
add_executable(server src/change_string_server.cpp)
ament_target_dependencies(server
rclcpp std_msgs)

install(TARGETS
  talker
  listener
  server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  set(TESTFILES test/main.cpp test/test.cpp)
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs)

  install(TARGETS
  test_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

endif()

rosidl_target_interfaces(server 
${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(talker 
${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
