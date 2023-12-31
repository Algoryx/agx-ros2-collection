# Created by Algoryx Simulation AB under the Apache-2.0 license.

cmake_minimum_required(VERSION 3.8)
project(agx_any_builder_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(
  agx_any_builder_parser
    "src/AnyMessageBuilder.cpp"
    "src/AnyMessageParser.cpp"
    "include/agx_any_builder_parser/AnyMessageBuilder.h"
    "include/agx_any_builder_parser/AnyMessageParser.h"
)

include_directories("include")

# Depends on agx_msgs containing the agx_msgs::Any message type.
find_package(agx_msgs REQUIRED)
ament_target_dependencies(agx_any_builder_parser agx_msgs)


ament_export_targets(agx_any_builder_parser HAS_LIBRARY_TARGET)

install(
  DIRECTORY "include/agx_any_builder_parser"
  DESTINATION "include"
)

install(
  TARGETS agx_any_builder_parser
  EXPORT agx_any_builder_parser
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
