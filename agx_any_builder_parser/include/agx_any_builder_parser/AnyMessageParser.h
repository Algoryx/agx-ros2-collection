// Created by Algoryx Simulation AB under the Apache-2.0 license.

#pragma once

#include "agx_msgs/msg/any.hpp"

// Standard libary includes.
#include <cstdint>
#include <vector>
#include <string>

/**
AnyMessageParser is a helper class for deserializing agxMsgs::Any back to the original custom
data type that was created using the AnyMessageBuilder.
*/
class AnyMessageParser
{
public:

  /** Must be called once prior to parsing a single complete message. */
  void beginParse();

  int8_t readInt8(const agx_msgs::msg::Any& message);
  uint8_t readUInt8(const agx_msgs::msg::Any& message);
  int16_t readInt16(const agx_msgs::msg::Any& message);
  uint16_t readUInt16(const agx_msgs::msg::Any& message);
  int32_t readInt32(const agx_msgs::msg::Any& message);
  uint32_t readUInt32(const agx_msgs::msg::Any& message);
  int64_t readInt64(const agx_msgs::msg::Any& message);
  uint64_t readUInt64(const agx_msgs::msg::Any& message);
  float readFloat32(const agx_msgs::msg::Any& message);
  double readDouble64(const agx_msgs::msg::Any& message);
  std::string readString(const agx_msgs::msg::Any& message);
  bool readBool(const agx_msgs::msg::Any& message);

  std::vector<int8_t> readInt8Sequence(const agx_msgs::msg::Any& message);
  std::vector<uint8_t> readUInt8Sequence(const agx_msgs::msg::Any& message);
  std::vector<int16_t> readInt16Sequence(const agx_msgs::msg::Any& message);
  std::vector<uint16_t> readUInt16Sequence(const agx_msgs::msg::Any& message);
  std::vector<int32_t> readInt32Sequence(const agx_msgs::msg::Any& message);
  std::vector<uint32_t> readUInt32Sequence(const agx_msgs::msg::Any& message);
  std::vector<int64_t> readInt64Sequence(const agx_msgs::msg::Any& message);
  std::vector<uint64_t> readUInt64Sequence(const agx_msgs::msg::Any& message);
  std::vector<float> readFloat32Sequence(const agx_msgs::msg::Any& message);
  std::vector<double> readDouble64Sequence(const agx_msgs::msg::Any& message);
  std::vector<std::string> readStringSequence(const agx_msgs::msg::Any& message);
  std::vector<bool> readBoolSequence(const agx_msgs::msg::Any& message);

private:
  size_t index{ 0 };
};
