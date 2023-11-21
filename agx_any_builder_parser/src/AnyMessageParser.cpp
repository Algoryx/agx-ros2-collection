// Created by Algoryx Simulation AB under the Apache-2.0 license.

#include "agx_any_builder_parser/AnyMessageParser.h"

namespace AnyMessageParser_helpers
{
  inline bool isBigEndian()
  {
    static constexpr uint16_t v = 1;
    static const bool bigEndian = (*((const uint8_t*)&v) == 0x0);
    return bigEndian;
  }

  template<typename TData>
  union ToType
  {    
    uint8_t buff[sizeof(TData)];
    TData t;
  };

  template <typename TData>
  TData read(const agx_msgs::msg::Any& message, size_t& outIndex)
  {
    ToType<TData> u;

    if (sizeof(TData) > static_cast<size_t>(std::numeric_limits<int64_t>::max()))
      throw std::invalid_argument("Invalid data size. The length is too large.");

    const int64_t size = static_cast<int64_t>(sizeof(TData));
    if (isBigEndian())
    {
      for (int64_t i = 0; i < size; i++)
      {
        u.buff[i] = message.data[outIndex++];
      }
    }
    else
    {
      for (int64_t i = size - 1; i >= 0; i--)
      {
        u.buff[i] = message.data[outIndex++];
      }
    }

    return u.t;
  }

  template <typename TData>
  std::vector<TData> readSequence(const agx_msgs::msg::Any& message, size_t& outIndex)
  {
    std::vector<TData> vec;
    const uint64_t length = read<uint64_t>(message, outIndex);
    vec.reserve(length);

    for (uint64_t i = 0; i < length; i++)
    {
      vec.push_back(read<TData>(message, outIndex));
    }

    return vec;
  }
}

void AnyMessageParser::beginParse()
{
  index = 0;
}


//
// Single member messages.
//

int8_t AnyMessageParser::readInt8(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<int8_t>(message, index);
}

uint8_t AnyMessageParser::readUInt8(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<uint8_t>(message, index);
}

int16_t AnyMessageParser::readInt16(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<int16_t>(message, index);
}

uint16_t AnyMessageParser::readUInt16(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<uint16_t>(message, index);
}

int32_t AnyMessageParser::readInt32(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<int32_t>(message, index);
}

uint32_t AnyMessageParser::readUInt32(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<uint32_t>(message, index);
}

int64_t AnyMessageParser::readInt64(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<int64_t>(message, index);
}

uint64_t AnyMessageParser::readUInt64(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<uint64_t>(message, index);
}


float AnyMessageParser::readFloat32(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<float>(message, index);
}

double AnyMessageParser::readDouble64(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<double>(message, index);
}

std::string AnyMessageParser::readString(const agx_msgs::msg::Any& message)
{
  const uint64_t length = AnyMessageParser_helpers::read<uint64_t>(message, index);
  std::string res;
  res.resize(length);

  for (uint64_t i = 0; i < length; i++)
  {
    res[i] = AnyMessageParser_helpers::read<char>(message, index);
  }

  return res;
}

bool AnyMessageParser::readBool(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::read<bool>(message, index);
}


//
// Multi-membered message types.
//
std::vector<int8_t> AnyMessageParser::readInt8Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<int8_t>(message, index);
}

std::vector<uint8_t> AnyMessageParser::readUInt8Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<uint8_t>(message, index);
}

std::vector<int16_t> AnyMessageParser::readInt16Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<int16_t>(message, index);
}

std::vector<uint16_t> AnyMessageParser::readUInt16Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<uint16_t>(message, index);
}

std::vector<int32_t> AnyMessageParser::readInt32Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<int32_t>(message, index);
}

std::vector<uint32_t> AnyMessageParser::readUInt32Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<uint32_t>(message, index);
}

std::vector<int64_t> AnyMessageParser::readInt64Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<int64_t>(message, index);
}

std::vector<uint64_t> AnyMessageParser::readUInt64Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<uint64_t>(message, index);
}

std::vector<float> AnyMessageParser::readFloat32Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<float>(message, index);
}

std::vector<double> AnyMessageParser::readDouble64Sequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<double>(message, index);
}

std::vector<std::string> AnyMessageParser::readStringSequence(const agx_msgs::msg::Any& message)
{
  std::vector<std::string> vec;
  const uint64_t length = AnyMessageParser_helpers::read<uint64_t>(message, index);
  vec.reserve(length);

  for (uint64_t i = 0; i < length; i++)
  {
    vec.push_back(readString(message));
  }

  return vec;
}

std::vector<bool> AnyMessageParser::readBoolSequence(const agx_msgs::msg::Any& message)
{
  return AnyMessageParser_helpers::readSequence<bool>(message, index);
}
