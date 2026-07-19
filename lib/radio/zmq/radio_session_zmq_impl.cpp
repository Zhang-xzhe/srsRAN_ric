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

#include "radio_session_zmq_impl.h"
#include <chrono>
#include <cstdio>
#include <mutex>

using namespace srsran;

namespace {
std::mutex& get_timing_log_mutex()
{
  static std::mutex m;
  return m;
}

FILE* get_timing_log_file()
{
  static FILE* f = []() {
    FILE* fp = std::fopen("/home/qijia/gnb_zmq_timing.csv", "w");
    if (fp != nullptr) {
      setlinebuf(fp);
      std::fprintf(fp, "time_us,samples_since_epoch,event\n");
    }
    return fp;
  }();
  return f;
}

/// Parses a socket type from a device argument string (e.g., "tx_type=push").
int parse_socket_type(const std::string& args, const char* key, int default_type)
{
  std::string prefix = std::string(key) + "=";
  size_t      pos    = args.find(prefix);
  if (pos == std::string::npos) {
    return default_type;
  }

  size_t value_start = pos + prefix.length();
  size_t value_end   = args.find(',', value_start);
  if (value_end == std::string::npos) {
    value_end = args.length();
  }

  std::string value = args.substr(value_start, value_end - value_start);
  if (value == "push") {
    return ZMQ_PUSH;
  }
  if (value == "pull") {
    return ZMQ_PULL;
  }
  if (value == "rep") {
    return ZMQ_REP;
  }
  if (value == "req") {
    return ZMQ_REQ;
  }
  if (value == "pub") {
    return ZMQ_PUB;
  }
  if (value == "sub") {
    return ZMQ_SUB;
  }

  return default_type;
}
} // namespace

radio_session_zmq_impl::radio_session_zmq_impl(const radio_configuration::radio& config,
                                               task_executor&                    async_task_executor,
                                               radio_notification_handler&       notifier) :
  logger(srslog::fetch_basic_logger("RF", false))
{
  // Make sure the number of streams are equal.
  srsran_assert(config.tx_streams.size() == config.rx_streams.size(),
                "The number of transmit streams (i.e., {}) must be equal to the number of receive streams (i.e., {}).",
                config.tx_streams.size(),
                config.rx_streams.size());

  // Make ZMQ context.
  zmq_context = zmq_ctx_new();
  if (zmq_context == nullptr) {
    logger.error("Failed to create ZMQ context. {}.", zmq_strerror(zmq_errno()));
    return;
  }

  unsigned nof_streams = config.tx_streams.size();

  // Parse ZMQ socket types from device arguments.
  int tx_socket_type = parse_socket_type(config.args, "tx_type", ZMQ_PUSH);
  int rx_socket_type = parse_socket_type(config.args, "rx_type", ZMQ_PULL);
  logger.info("ZMQ socket types: tx_type={}, rx_type={}", tx_socket_type, rx_socket_type);

  // Store sampling rate and time reference.
  srate_hz           = config.sampling_rate_Hz;
  session_start_time = std::chrono::system_clock::now();

  // Debug log level is only available if verbose keyword is in the device arguments.
  bool allow_log_level_debug = (config.args.find("verbose") != std::string::npos);

  // ZMQ logging in debug is extremely verbose. The following lines avoid debug level unless set to paranoid.
  srslog::basic_levels log_level = config.log_level;
  if (!allow_log_level_debug && (log_level >= srslog::basic_levels::debug)) {
    log_level = srslog::basic_levels::info;
  }

  // Iterate for each transmission and reception stream.
  for (unsigned stream_id = 0; stream_id != nof_streams; ++stream_id) {
    const radio_configuration::stream& tx_radio_stream_config = config.tx_streams[stream_id];

    // Prepare transmit stream configuration.
    radio_zmq_tx_stream::stream_description tx_stream_config;
    tx_stream_config.socket_type = tx_socket_type;
    for (unsigned channel_id = 0; channel_id != tx_radio_stream_config.channels.size(); ++channel_id) {
      tx_stream_config.address.push_back(tx_radio_stream_config.channels[channel_id].args);
    }
    tx_stream_config.stream_id         = stream_id;
    tx_stream_config.stream_id_str     = "zmq:tx:" + std::to_string(stream_id);
    tx_stream_config.log_level         = log_level;
    tx_stream_config.trx_timeout_ms    = DEFAULT_TRX_TIMEOUT_MS;
    tx_stream_config.linger_timeout_ms = DEFAULT_LINGER_TIMEOUT_MS;
    tx_stream_config.buffer_size       = DEFAULT_STREAM_BUFFER_SIZE;

    const radio_configuration::stream& rx_radio_stream_config = config.rx_streams[stream_id];

    // Prepare receive stream configuration.
    radio_zmq_rx_stream::stream_description rx_stream_config;
    rx_stream_config.socket_type = rx_socket_type;
    for (unsigned channel_id = 0; channel_id != rx_radio_stream_config.channels.size(); ++channel_id) {
      rx_stream_config.address.push_back(rx_radio_stream_config.channels[channel_id].args);
    }
    rx_stream_config.stream_id         = stream_id;
    rx_stream_config.stream_id_str     = "zmq:rx:" + std::to_string(stream_id);
    rx_stream_config.log_level         = log_level;
    rx_stream_config.trx_timeout_ms    = DEFAULT_TRX_TIMEOUT_MS;
    rx_stream_config.linger_timeout_ms = DEFAULT_LINGER_TIMEOUT_MS;
    rx_stream_config.buffer_size       = DEFAULT_STREAM_BUFFER_SIZE;

    // Create baseband gateway.
    bb_gateways.emplace_back(std::make_unique<radio_zmq_baseband_gateway>(
        zmq_context, async_task_executor, notifier, tx_stream_config, rx_stream_config));

    // Make sure streams are created successfully.
    if (!bb_gateways.back()->get_tx_stream().is_successful() || !bb_gateways.back()->get_rx_stream().is_successful()) {
      return;
    }
  }

  successful = true;
}

radio_session_zmq_impl::~radio_session_zmq_impl()
{
  // Destroy transmit and receive streams prior to ZMQ context destruction.
  bb_gateways.clear();

  // Destroy ZMQ context.
  if (zmq_context != nullptr) {
    zmq_ctx_shutdown(zmq_context);
    zmq_ctx_destroy(zmq_context);
    zmq_context = nullptr;
  }
}

void radio_session_zmq_impl::stop()
{
  // Signal stop for each transmit stream.
  for (auto& gateway : bb_gateways) {
    gateway->get_tx_stream().stop();
  }

  // Signal stop for each receive stream.
  for (auto& gateway : bb_gateways) {
    gateway->get_rx_stream().stop();
  }

  // Wait for streams to join.
  for (auto& gateway : bb_gateways) {
    gateway->get_tx_stream().wait_stop();
    gateway->get_rx_stream().wait_stop();
  }
}

bool radio_session_zmq_impl::set_tx_gain(unsigned port_id, double gain_dB)
{
  return false;
}

bool radio_session_zmq_impl::set_rx_gain(unsigned port_id, double gain_dB)
{
  return false;
}

void radio_session_zmq_impl::start(baseband_gateway_timestamp start_time)
{
  for (auto& gateway : bb_gateways) {
    gateway->get_rx_stream().start(start_time);
  }
  for (auto& gateway : bb_gateways) {
    gateway->get_tx_stream().start(start_time);
  }
}

baseband_gateway_timestamp radio_session_zmq_impl::read_current_time()
{
  auto now = std::chrono::system_clock::now();
  auto us  = std::chrono::duration_cast<std::chrono::microseconds>(now - session_start_time).count();
  baseband_gateway_timestamp ts = static_cast<baseband_gateway_timestamp>(us * srate_hz / 1e6);

  // Diagnostic logging: sample every 1000th call to avoid overhead.
  static unsigned counter = 0;
  if ((counter++ % 1000) == 0) {
    std::lock_guard<std::mutex> lock(get_timing_log_mutex());
    FILE* f = get_timing_log_file();
    if (f != nullptr) {
      auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
      std::fprintf(f, "%ld,%lu,read_current_time\n", wall_us, ts);
    }
  }

  return ts;
}
bool radio_session_zmq_impl::set_tx_freq(unsigned stream_id, double center_freq_Hz)
{
  return false;
}
bool radio_session_zmq_impl::set_rx_freq(unsigned stream_id, double center_freq_Hz)
{
  return false;
}
