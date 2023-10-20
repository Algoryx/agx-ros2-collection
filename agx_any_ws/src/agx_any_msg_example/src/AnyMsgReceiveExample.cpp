// Created by Algoryx Simulation AB under the Apache-2.0 license.

#include "agx_msgs/msg/Any.hpp"
#include "agx_any_builder_parser/AnyMessageParser.h"

#include "rclcpp/rclcpp.hpp"

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

using std::placeholders::_1;
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

class AnySubscriber : public rclcpp::Node
{
  public:
    AnySubscriber()
    : Node("any_subscriber")
    {
      subscription_ = this->create_subscription<agx_msgs::msg::Any>(
      "agx_any", 10, std::bind(&AnySubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const agx_msgs::msg::Any::SharedPtr msg)
    {
      // We can deserialize the agx_msgs::Any message back to our custom struct type using
      // the AnyMessageParser. Notice that the order of reading back the data matches the custom
      // struct layout, this is important.
      parser_.beginParse();
      myStruct_.myInt = parser_.readInt32(*msg);
      myStruct_.myStrings = parser_.readStringSequence(*msg);
      myStruct_.myBool = parser_.readBool(*msg);

      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", to_string(myStruct_).c_str());
    }

    rclcpp::Subscription<agx_msgs::msg::Any>::SharedPtr subscription_;
    MyStruct myStruct_;
    AnyMessageParser parser_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnySubscriber>());
  rclcpp::shutdown();
  return 0;
}
