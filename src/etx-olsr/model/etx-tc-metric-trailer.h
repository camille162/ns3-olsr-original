/*
 * Optional UDP payload trailer: per-TC link ETX sidecar (ETX-OLSR only).
 * Lives after standard OLSR messages; OLSR PacketHeader length excludes this trailer.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ETX_TC_METRIC_TRAILER_H
#define ETX_TC_METRIC_TRAILER_H

#include "ns3/ipv4-address.h"
#include "ns3/packet.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace ns3
{
namespace etxolsr
{

/// Magic 'ETX1' — identifies the trailer; vanilla OLSR stack ignores trailing bytes.
constexpr uint32_t TC_ETX_TRAILER_MAGIC = 0x45545831;

/**
 * One TC message's advertised neighbor-main → link ETX from tcOriginator.
 */
struct TcEtxTrailerBlock
{
  Ipv4Address tcOriginator;
  uint16_t messageSequenceNumber{0};
  uint16_t ansn{0};
  std::vector<std::pair<Ipv4Address, double>> neighborEtx;
};

/** Serialize blocks to a Packet, or empty Ptr if \p blocks is empty. */
Ptr<Packet> SerializeTcEtxTrailer(const std::vector<TcEtxTrailerBlock>& blocks);

/**
 * If \p packet starts with TC_ETX_TRAILER_MAGIC, parse trailer, append to \p out,
 * and RemoveAtEnd() those bytes from \p packet. Otherwise leave \p packet unchanged.
 * @return true if a valid trailer was consumed.
 */
bool TryConsumeTcEtxTrailer(Ptr<Packet> packet, std::vector<TcEtxTrailerBlock>& out);

/** Build trailer from parallel optional blocks (same order as batched messages). */
Ptr<Packet> SerializeTcEtxTrailerFromOptionals(
    const std::vector<std::optional<TcEtxTrailerBlock>>& optionals);

} // namespace etxolsr
} // namespace ns3

#endif /* ETX_TC_METRIC_TRAILER_H */
