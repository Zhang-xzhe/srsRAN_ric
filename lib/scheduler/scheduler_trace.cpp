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

#include "srsran/scheduler/scheduler_trace.h"
#include "srsran/ran/slot_point.h"
#include "srsran/srslog/srslog.h"
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace srsran;

dl_scheduler_trace_manager::dl_scheduler_trace_manager(const std::string& trace_file)
{
  if (!trace_file.empty()) {
    load_trace_file(trace_file);
  }
}

void dl_scheduler_trace_manager::load_trace_file(const std::string& filename)
{
  auto&         logger = srslog::fetch_basic_logger("SCHED");
  std::ifstream file(filename);
  if (!file.is_open()) {
    logger.warning("Failed to open scheduler trace file '{}'. Trace-based scheduling disabled.", filename);
    enabled_ = false;
    return;
  }

  std::string line;
  size_t      line_num = 0, valid_samples = 0;
  while (std::getline(file, line)) {
    ++line_num;
    if (line.empty() || line[0] == '#') {
      continue;
    }
    dl_scheduler_trace_sample sample;
    if (parse_trace_line(line, sample)) {
      slot_to_index_[sample.slot_index] = trace_samples_.size();
      trace_samples_.push_back(sample);
      ++valid_samples;
    } else {
      logger.warning("Failed to parse trace line {} in file '{}'", line_num, filename);
    }
  }

  if (trace_samples_.empty()) {
    logger.warning("No valid samples in trace file '{}'. Trace-based scheduling disabled.", filename);
    enabled_ = false;
  } else {
    logger.info("Loaded {} scheduler trace samples from '{}' ({} lines processed)", valid_samples, filename, line_num);
    enabled_ = true;
  }
}

bool dl_scheduler_trace_manager::parse_trace_line(const std::string& line, dl_scheduler_trace_sample& sample)
{
  std::istringstream iss(line);
  char               comma;
  int                needs_retx_int;
  int                harq_id_int = -1;

  // 统计逗号数量来判断格式
  size_t comma_count = std::count(line.begin(), line.end(), ',');

  if (comma_count == 1) {
    // TBS-only 格式：slot_index,tbs_bytes
    iss >> sample.slot_index >> comma >> sample.tbs;
    if (iss.fail()) {
      return false;
    }
    sample.mcs        = sch_mcs_index{0};
    sample.needs_retx = false;
    sample.retx_count = 0;
    sample.harq_id    = std::nullopt;
  } else {
    // 完整格式：slot_index,mcs,tbs_bytes,needs_retx,retx_count[,harq_id]
    unsigned mcs_val;
    iss >> sample.slot_index >> comma >> mcs_val >> comma >> sample.tbs >> comma >> needs_retx_int >> comma >>
        sample.retx_count;
    sample.mcs = sch_mcs_index{static_cast<uint8_t>(mcs_val)};
    if (iss.fail()) {
      return false;
    }
    if (iss >> comma >> harq_id_int && harq_id_int >= 0) {
      sample.harq_id = to_harq_id(harq_id_int);
    }
    sample.needs_retx = (needs_retx_int != 0);
    if (sample.mcs > 28) {
      return false;
    }
  }

  if (sample.tbs == 0 || sample.retx_count > 4) {
    return false;
  }
  return true;
}

std::optional<dl_scheduler_trace_sample>
dl_scheduler_trace_manager::get_trace_sample(slot_point slot) const
{
  if (!enabled_ || trace_samples_.empty()) {
    return std::nullopt;
  }

  // 达到 start_slot_ 之前不应用 trace（保护 UE 接入）
  if (slot.to_uint() < start_slot_ && !already_access) {
    already_access = true;
    return std::nullopt;
  }

  // 根据经过的时隙数推进 current_index_（时间驱动）
  if (last_access_slot_.has_value()) {
    unsigned slots_elapsed = slot.to_uint() - last_access_slot_->to_uint();
    current_index_         = (current_index_ + slots_elapsed) % trace_samples_.size();
  }
  last_access_slot_ = slot;

  return trace_samples_[current_index_];
}

std::optional<dl_scheduler_trace_sample>
dl_scheduler_trace_manager::get_sample_by_index(size_t index) const
{
  if (!enabled_ || trace_samples_.empty() || index >= trace_samples_.size()) {
    return std::nullopt;
  }
  return trace_samples_[index];
}
