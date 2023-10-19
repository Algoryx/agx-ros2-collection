# Created by Algoryx Simulation AB under the Apache-2.0 license.

cmake_minimum_required(VERSION 3.8)
project(agx_any_builder_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(
  agx_any_builder_parser
    "src/AnyMessageBuilder.cpp"
    "src/AnyMessageParser.cpp"
    "include/agx_any_builder_parser/AnyMessageBuilder.h"
    "include/agx_any_builder_parser/AnyMessageParser.h"
)

target_include_directories(agx_any_builder_parser PUBLIC "include")


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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
