# Created by Algoryx Simulation AB under the Apache-2.0 license.

cmake_minimum_required(VERSION 3.8)
project(agx_any_msg_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sender "src/AnyMsgSendExample.cpp")
add_executable(receiver "src/AnyMsgReceiveExample.cpp")

# Depends on agx_msgs and agx_any_builder_parser that will be used
# to send and receive custom data types.
find_package(agx_msgs REQUIRED)
find_package(agx_any_builder_parser REQUIRED)

ament_target_dependencies(
  sender
    rclcpp
    agx_msgs
    agx_any_builder_parser
)

ament_target_dependencies(
  receiver
    rclcpp
    agx_msgs
    agx_any_builder_parser
)

install(TARGETS sender receiver DESTINATION lib/${PROJECT_NAME})

ament_package()
