/*
 *
 * Copyright 2021-2025 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#pragma once

#include "sdap_session_logger.h"
#include "srsran/sdap/sdap.h"

namespace srsran {

namespace srs_cu_up {

class sdap_entity_tx_impl
{
public:
  sdap_entity_tx_impl(uint32_t              ue_index,
                      pdu_session_id_t      psi,
                      qos_flow_id_t         qfi_,
                      drb_id_t              drb_id_,
                      sdap_tx_pdu_notifier& pdu_notifier_) :
    logger("SDAP", {ue_index, psi, qfi_, drb_id_, "DL"}), qfi(qfi_), drb_id(drb_id_), pdu_notifier(pdu_notifier_)
  {
  }

  void handle_sdu(byte_buffer sdu)
  {
    // pass through
    logger.log_debug("TX PDU. {} pdu_len={}", qfi, sdu.length());

    // Log IP 5-tuple for TCP/UDP packets
    if (sdu.length() >= 1) {
      const uint8_t version = (sdu[0] >> 4U) & 0x0fU;
      if (version == 4 && sdu.length() >= 20) {
        const uint8_t ihl   = (sdu[0] & 0x0fU) * 4U;
        const uint8_t proto = sdu[9];
        if (ihl >= 20 && (proto == 6 || proto == 17) && sdu.length() >= static_cast<size_t>(ihl) + 4U) {
          const uint16_t src_port = (static_cast<uint16_t>(sdu[ihl]) << 8U) | sdu[ihl + 1U];
          const uint16_t dst_port = (static_cast<uint16_t>(sdu[ihl + 2U]) << 8U) | sdu[ihl + 3U];
          if (proto == 6 && sdu.length() >= static_cast<size_t>(ihl) + 12U) {
            const uint32_t seq = (static_cast<uint32_t>(sdu[ihl + 4U]) << 24U) |
                                 (static_cast<uint32_t>(sdu[ihl + 5U]) << 16U) |
                                 (static_cast<uint32_t>(sdu[ihl + 6U]) << 8U) |
                                  static_cast<uint32_t>(sdu[ihl + 7U]);
            const uint32_t ack = (static_cast<uint32_t>(sdu[ihl + 8U]) << 24U) |
                                 (static_cast<uint32_t>(sdu[ihl + 9U]) << 16U) |
                                 (static_cast<uint32_t>(sdu[ihl + 10U]) << 8U) |
                                  static_cast<uint32_t>(sdu[ihl + 11U]);
            logger.log_debug("5-tuple: {}.{}.{}.{}:{} -> {}.{}.{}.{}:{} proto=TCP seq={} ack={}",
                             +sdu[12], +sdu[13], +sdu[14], +sdu[15], src_port,
                             +sdu[16], +sdu[17], +sdu[18], +sdu[19], dst_port,
                             seq, ack);
          } else {
            logger.log_debug("5-tuple: {}.{}.{}.{}:{} -> {}.{}.{}.{}:{} proto=UDP",
                             +sdu[12], +sdu[13], +sdu[14], +sdu[15], src_port,
                             +sdu[16], +sdu[17], +sdu[18], +sdu[19], dst_port);
          }
        }
      } else if (version == 6 && sdu.length() >= 44) {
        const uint8_t next_hdr = sdu[6];
        if (next_hdr == 6 || next_hdr == 17) {
          const uint16_t src_port = (static_cast<uint16_t>(sdu[40]) << 8U) | sdu[41];
          const uint16_t dst_port = (static_cast<uint16_t>(sdu[42]) << 8U) | sdu[43];
          if (next_hdr == 6 && sdu.length() >= 52U) {
            const uint32_t seq = (static_cast<uint32_t>(sdu[44]) << 24U) | (static_cast<uint32_t>(sdu[45]) << 16U) |
                                 (static_cast<uint32_t>(sdu[46]) << 8U)  |  static_cast<uint32_t>(sdu[47]);
            const uint32_t ack = (static_cast<uint32_t>(sdu[48]) << 24U) | (static_cast<uint32_t>(sdu[49]) << 16U) |
                                 (static_cast<uint32_t>(sdu[50]) << 8U)  |  static_cast<uint32_t>(sdu[51]);
            logger.log_debug(
                "5-tuple: [{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}]:{}"
                " -> [{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}]:{} proto=TCP seq={} ack={}",
                +sdu[8],  +sdu[9],  +sdu[10], +sdu[11], +sdu[12], +sdu[13], +sdu[14], +sdu[15],
                +sdu[16], +sdu[17], +sdu[18], +sdu[19], +sdu[20], +sdu[21], +sdu[22], +sdu[23], src_port,
                +sdu[24], +sdu[25], +sdu[26], +sdu[27], +sdu[28], +sdu[29], +sdu[30], +sdu[31],
                +sdu[32], +sdu[33], +sdu[34], +sdu[35], +sdu[36], +sdu[37], +sdu[38], +sdu[39], dst_port,
                seq, ack);
          } else {
            logger.log_debug(
                "5-tuple: [{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}]:{}"
                " -> [{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}:{:02x}{:02x}]:{} proto=UDP",
                +sdu[8],  +sdu[9],  +sdu[10], +sdu[11], +sdu[12], +sdu[13], +sdu[14], +sdu[15],
                +sdu[16], +sdu[17], +sdu[18], +sdu[19], +sdu[20], +sdu[21], +sdu[22], +sdu[23], src_port,
                +sdu[24], +sdu[25], +sdu[26], +sdu[27], +sdu[28], +sdu[29], +sdu[30], +sdu[31],
                +sdu[32], +sdu[33], +sdu[34], +sdu[35], +sdu[36], +sdu[37], +sdu[38], +sdu[39], dst_port);
          }
        }
      }
    }

    pdu_notifier.on_new_pdu(std::move(sdu));
  }

  drb_id_t get_drb_id() const { return drb_id; }

private:
  sdap_session_trx_logger logger;
  qos_flow_id_t           qfi;
  drb_id_t                drb_id;
  sdap_tx_pdu_notifier&   pdu_notifier;
};

} // namespace srs_cu_up

} // namespace srsran
