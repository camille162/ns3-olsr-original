/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "etx-tc-metric-trailer.h"

#include <cstring>
#include <vector>

namespace ns3
{
namespace etxolsr
{
namespace
{

void
AppendBe32(std::vector<uint8_t>& raw, uint32_t v)
{
  raw.push_back(static_cast<uint8_t>((v >> 24) & 0xff));
  raw.push_back(static_cast<uint8_t>((v >> 16) & 0xff));
  raw.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
  raw.push_back(static_cast<uint8_t>(v & 0xff));
}

void
AppendBe16(std::vector<uint8_t>& raw, uint16_t v)
{
  raw.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
  raw.push_back(static_cast<uint8_t>(v & 0xff));
}

uint32_t
ReadBe32(const std::vector<uint8_t>& raw, uint32_t& off)
{
  uint32_t v = (uint32_t(raw[off]) << 24) | (uint32_t(raw[off + 1]) << 16) |
               (uint32_t(raw[off + 2]) << 8) | uint32_t(raw[off + 3]);
  off += 4;
  return v;
}

uint16_t
ReadBe16(const std::vector<uint8_t>& raw, uint32_t& off)
{
  uint16_t v = (uint16_t(raw[off]) << 8) | uint16_t(raw[off + 1]);
  off += 2;
  return v;
}

void
AppendNboDouble(std::vector<uint8_t>& raw, double value)
{
  uint64_t u = 0;
  static_assert(sizeof(double) == sizeof(uint64_t), "double size");
  std::memcpy(&u, &value, sizeof(double));
  AppendBe32(raw, static_cast<uint32_t>((u >> 32) & 0xffffffffu));
  AppendBe32(raw, static_cast<uint32_t>(u & 0xffffffffu));
}

double
ReadNboDouble(const std::vector<uint8_t>& raw, uint32_t& off)
{
  const uint64_t hi = ReadBe32(raw, off);
  const uint64_t lo = ReadBe32(raw, off);
  const uint64_t u = (hi << 32) | lo;
  double value = 0.0;
  std::memcpy(&value, &u, sizeof(double));
  return value;
}

} // namespace

Ptr<Packet>
SerializeTcEtxTrailer(const std::vector<TcEtxTrailerBlock>& blocks)
{
  if (blocks.empty())
  {
    return nullptr;
  }
  std::vector<uint8_t> raw;
  raw.reserve(256);
  AppendBe32(raw, TC_ETX_TRAILER_MAGIC);
  AppendBe16(raw, 1); // version
  AppendBe16(raw, static_cast<uint16_t>(blocks.size()));
  for (const auto& b : blocks)
  {
    AppendBe32(raw, b.tcOriginator.Get());
    AppendBe16(raw, b.messageSequenceNumber);
    AppendBe16(raw, b.ansn);
    AppendBe16(raw, static_cast<uint16_t>(b.neighborEtx.size()));
    for (const auto& ne : b.neighborEtx)
    {
      AppendBe32(raw, ne.first.Get());
      AppendNboDouble(raw, ne.second);
    }
  }
  return Create<Packet>(raw.data(), static_cast<uint32_t>(raw.size()));
}

bool
TryConsumeTcEtxTrailer(Ptr<Packet> packet, std::vector<TcEtxTrailerBlock>& out)
{
  out.clear();
  if (!packet || packet->GetSize() < 8)
  {
    return false;
  }
  const uint32_t n = packet->GetSize();
  std::vector<uint8_t> raw(n);
  packet->CopyData(raw.data(), n);
  uint32_t off = 0;
  if (ReadBe32(raw, off) != TC_ETX_TRAILER_MAGIC)
  {
    return false;
  }
  if (ReadBe16(raw, off) != 1)
  {
    return false;
  }
  const uint16_t nblk = ReadBe16(raw, off);
  std::vector<TcEtxTrailerBlock> blocks;
  blocks.reserve(nblk);
  for (uint16_t bi = 0; bi < nblk; ++bi)
  {
    if (off + 4 + 2 + 2 + 2 > n)
    {
      return false;
    }
    TcEtxTrailerBlock block;
    block.tcOriginator = Ipv4Address(ReadBe32(raw, off));
    block.messageSequenceNumber = ReadBe16(raw, off);
    block.ansn = ReadBe16(raw, off);
    const uint16_t nn = ReadBe16(raw, off);
    if (off + uint32_t(nn) * (4u + 8u) > n)
    {
      return false;
    }
    block.neighborEtx.reserve(nn);
    for (uint16_t j = 0; j < nn; ++j)
    {
      const Ipv4Address a(ReadBe32(raw, off));
      const double etx = ReadNboDouble(raw, off);
      block.neighborEtx.emplace_back(a, etx);
    }
    blocks.push_back(std::move(block));
  }
  if (off != n)
  {
    return false;
  }
  out = std::move(blocks);
  packet->RemoveAtEnd(n);
  return true;
}

Ptr<Packet>
SerializeTcEtxTrailerFromOptionals(const std::vector<std::optional<TcEtxTrailerBlock>>& optionals)
{
  std::vector<TcEtxTrailerBlock> blocks;
  for (const auto& o : optionals)
  {
    if (o)
    {
      blocks.push_back(*o);
    }
  }
  return SerializeTcEtxTrailer(blocks);
}

} // namespace etxolsr
} // namespace ns3
