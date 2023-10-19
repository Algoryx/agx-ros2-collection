// Created by Algoryx Simulation AB under the Apache-2.0 license.

#pragma once

#include "agx_msgs/msg/Any.hpp"

// Standard library includes.
#include <cstdint>
#include <memory>
#include <string>
#include <vector>


/**
AnyMessageBuilder is a helper class for serializing custom data structures to an agxMsgs::Any
message type that can then be sent via ROS2 and deserialized using the AnyMessageParser.
*/
class AnyMessageBuilder
{
public:
  AnyMessageBuilder();
  AnyMessageBuilder(const AnyMessageBuilder& other) = delete;
  AnyMessageBuilder(AnyMessageBuilder&& other) noexcept;
  AnyMessageBuilder operator=(const AnyMessageBuilder& other) = delete;
  AnyMessageBuilder& operator=(AnyMessageBuilder&& other) noexcept;
  ~AnyMessageBuilder();

  AnyMessageBuilder& beginMessage();

  AnyMessageBuilder& writeInt8(int8_t d);
  AnyMessageBuilder& writeUInt8(uint8_t d);
  AnyMessageBuilder& writeInt16(int16_t d);
  AnyMessageBuilder& writeUInt16(uint16_t d);
  AnyMessageBuilder& writeInt32(int32_t d);
  AnyMessageBuilder& writeUInt32(uint32_t d);
  AnyMessageBuilder& writeInt64(int64_t d);
  AnyMessageBuilder& writeUInt64(uint64_t d);
  AnyMessageBuilder& writeFloat32(float d);
  AnyMessageBuilder& writeDouble64(double d);
  AnyMessageBuilder& writeString(const std::string& d);
  AnyMessageBuilder& writeBool(bool d);

  AnyMessageBuilder& writeInt8Sequence(const std::vector<int8_t>& d);
  AnyMessageBuilder& writeUInt8Sequence(const std::vector<uint8_t>& d);
  AnyMessageBuilder& writeInt16Sequence(const std::vector<int16_t>& d);
  AnyMessageBuilder& writeUInt16Sequence(const std::vector<uint16_t>& d);
  AnyMessageBuilder& writeInt32Sequence(const std::vector<int32_t>& d);
  AnyMessageBuilder& writeUInt32Sequence(const std::vector<uint32_t>& d);
  AnyMessageBuilder& writeInt64Sequence(const std::vector<int64_t>& d);
  AnyMessageBuilder& writeUInt64Sequence(const std::vector<uint64_t>& d);
  AnyMessageBuilder& writeFloat32Sequence(const std::vector<float>& d);
  AnyMessageBuilder& writeDouble64Sequence(const std::vector<double>& d);
  AnyMessageBuilder& writeStringSequence(const std::vector<std::string>& d);
  AnyMessageBuilder& writeBoolSequence(const std::vector<bool>& d);

  const agx_msgs::msg::Any& getMessage() const;

  void reserveBytes(size_t bytes);

private:
  std::unique_ptr<agx_msgs::msg::Any> message;
};
