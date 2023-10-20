// Created by Algoryx Simulation AB under the Apache-2.0 license.

#include "agx_any_builder_parser/AnyMessageBuilder.h"

// Standard library includes.
#include <limits>
#include <stdexcept>


namespace AnyMessageBuilder_helpers
{
  inline bool isBigEndian()
  {
    static constexpr uint16_t v = 1;
    static const bool bigEndian = (*((const uint8_t*)&v) == 0x0);
    return bigEndian;
  }

  template <typename TData>
  union FromType
  {
    TData f;
    uint8_t buff[sizeof(TData)];
  };

  template <typename TData>
  void write(const TData& data, agx_msgs::msg::Any& outMessage)
  {
    FromType<TData> u;
    u.f = data;

    if (isBigEndian())
    {
      for (int64_t i = 0; i < sizeof(TData); i++)
      {
        outMessage.data.push_back(u.buff[i]);
      }
    }
    else
    {
      for (int64_t i = sizeof(TData) - 1; i >= 0; i--)
      {
        outMessage.data.push_back(u.buff[i]);
      }
    }
  }

  template <>
  void write<std::string>(const std::string& data, agx_msgs::msg::Any& outMessage)
  {
    for (const auto& c : data)
      write(c, outMessage);
  }

  template <typename TData>
  void writeSequence(const std::vector<TData>& data, agx_msgs::msg::Any& outMessage)
  {
    if (data.size() > std::numeric_limits<uint64_t>::max())
      throw std::invalid_argument("Invalid vector length. The length is too large");

    // We write the length of the vector to the data buffer of the message so that the
    // parser can determine the number of elements in the vector on the receiving side. 
    write(static_cast<uint64_t>(data.size()), outMessage);

    for (const TData& d : data)
      write(d, outMessage);
  }
}

AnyMessageBuilder::AnyMessageBuilder() :
  message(std::make_unique<agx_msgs::msg::Any>())
{
}

AnyMessageBuilder::AnyMessageBuilder(AnyMessageBuilder&& other) noexcept
{
  message = std::move(other.message);
}

AnyMessageBuilder& AnyMessageBuilder::operator=(AnyMessageBuilder&& other) noexcept
{
  message = std::move(other.message);
  return *this;
}

AnyMessageBuilder::~AnyMessageBuilder()
{
}

void AnyMessageBuilder::reserveBytes(size_t bytes)
{
  // Underlying data is one byte in size.
  message->data.reserve(bytes);
}

AnyMessageBuilder& AnyMessageBuilder::beginMessage()
{
  message->data.clear();
  return *this;
}



//
// Single member types.
//

AnyMessageBuilder& AnyMessageBuilder::writeInt8(int8_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt8(uint8_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeInt16(int16_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt16(uint16_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeInt32(int32_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt32(uint32_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeInt64(int64_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt64(uint64_t d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}


AnyMessageBuilder& AnyMessageBuilder::writeFloat32(float d)
{
  static_assert(sizeof(float) == 4, "Unexpected sizeof float in AnyMessageBuilder. "
    "This platform will not be supported.");
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeDouble64(double d)
{
  static_assert(sizeof(double) == 8, "Unexpected sizeof double in AnyMessageBuilder."
    "This platform will not be supported.");
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeString(const std::string& d)
{
  // We write the length of the string to the data buffer of the message so
  // that the parser can determine the number of chars in the string.
  AnyMessageBuilder_helpers::write(static_cast<uint64_t>(d.size()), *message);

  // Write the actual chars.
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeBool(bool d)
{
  AnyMessageBuilder_helpers::write(d, *message);
  return *this;
}


//
// Sequence types.
//

AnyMessageBuilder& AnyMessageBuilder::writeInt8Sequence(const std::vector<int8_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt8Sequence(const std::vector<uint8_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeInt16Sequence(const std::vector<int16_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt16Sequence(const std::vector<uint16_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeInt32Sequence(const std::vector<int32_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt32Sequence(const std::vector<uint32_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeInt64Sequence(const std::vector<int64_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeUInt64Sequence(const std::vector<uint64_t>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeFloat32Sequence(const std::vector<float>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeDouble64Sequence(const std::vector<double>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeStringSequence(const std::vector<std::string>& d)
{
  // We write the length of the vector to the data buffer of the message so
  // that the parser can determine the number of strings in the vector.
  AnyMessageBuilder_helpers::write(static_cast<uint64_t>(d.size()), *message);

  for (const auto& str : d)
    writeString(str);
  return *this;
}

AnyMessageBuilder& AnyMessageBuilder::writeBoolSequence(const std::vector<bool>& d)
{
  AnyMessageBuilder_helpers::writeSequence(d, *message);
  return *this;
}


const agx_msgs::msg::Any& AnyMessageBuilder::getMessage() const
{
  return *message;
}
