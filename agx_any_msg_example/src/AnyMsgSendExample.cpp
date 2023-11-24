// Created by Algoryx Simulation AB under the Apache-2.0 license.

#include "agx_msgs/msg/any.hpp"
#include "agx_any_builder_parser/AnyMessageBuilder.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

struct MyStruct
{
    int32_t myInt {0};
    std::vector<std::string> myStrings;
    bool myBool {false};
};

std::string to_string(const MyStruct& v)
{
    std::stringstream ss;
    ss << v.myInt << " " << "{ ";
    for (const auto& s : v.myStrings)
        ss << s << ", ";

    auto boolToStr = [](bool b) { return b ? "true" : "false"; };
    ss << "} " << boolToStr(v.myBool);
    return ss.str();
}

class AnyPublisher : public rclcpp::Node
{
  public:
    AnyPublisher()
    : Node("any_publisher")
    {
      publisher_ = this->create_publisher<agx_msgs::msg::Any>("agx_any", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&AnyPublisher::timer_callback, this));

      // Set data to the custom struct that is about to be sent.
      myStruct_.myInt = 314;
      myStruct_.myStrings.push_back("First string");
      myStruct_.myStrings.push_back("Second string");
      myStruct_.myBool = true;
    }

  private:
    void timer_callback()
    {
      // Serialize our custom struct as an agx_msgs::Any message using the AnyMessageBuilder.
      // Notice that the order of building the message matches the custom struct layout.
      builder_
        .beginMessage()
        .writeInt32(myStruct_.myInt)
        .writeStringSequence(myStruct_.myStrings)
        .writeBool(myStruct_.myBool);
                
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", to_string(myStruct_).c_str());

      // By calling AnyMessageBuilder::getMessage we get an agx_msgs::Any that is our built
      // custom message. We then send it using our publisher.
      publisher_->publish(builder_.getMessage());
    }

    AnyMessageBuilder builder_;
    MyStruct myStruct_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<agx_msgs::msg::Any>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnyPublisher>());
  rclcpp::shutdown();
  return 0;
}
