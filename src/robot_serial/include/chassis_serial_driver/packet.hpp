#ifndef CHASSIS_SERIAL_DRIVER__PACKET_HPP_
#define CHASSIS_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace chassis_serial_driver
{

struct ReceivePacket
{
  uint8_t frame_header[2];
  float x;
  float y;

} __attribute__((packed));

struct ReceivePacket_1
{
  uint8_t frame_header[2];
  uint8_t xipan_zhuangtai;
  uint8_t taijie_zhuangtai;
  uint8_t up_free;
  uint8_t down_free;
} __attribute__((packed));

struct SendPacket
{
  uint8_t frame_header[2] = {0xAA, 0x55};
  float x1;
  float y1;
  float yaw1;
  float x;
  float y;
  float yaw;
  uint8_t spearhead;
  uint8_t block;
  uint8_t stair;
  uint8_t area;
  uint8_t dt35;

} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
};

inline ReceivePacket_1 fromVector_1(const std::vector<uint8_t> & data)
{
  ReceivePacket_1 packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
};

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
};

}  // namespace chassis_serial_driver
#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
