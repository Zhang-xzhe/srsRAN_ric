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

#include "srsran/ran/sch/sch_mcs.h"
#include "srsran/ran/slot_point.h"
#include "srsran/scheduler/harq_id.h"
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace srsran {

/// 单时隙的 trace 样本
struct dl_scheduler_trace_sample {
  unsigned                 slot_index;  ///< Trace 内的时隙索引
  sch_mcs_index            mcs;         ///< MCS 索引（TBS-only 格式时设为 0）
  unsigned                 tbs;         ///< 传输块大小（字节）
  bool                     needs_retx;  ///< 是否为重传
  unsigned                 retx_count;  ///< 重传次数（0 = 首次）
  std::optional<harq_id_t> harq_id;    ///< HARQ 进程 ID（可选）

  dl_scheduler_trace_sample() : slot_index(0), mcs(0), tbs(0), needs_retx(false), retx_count(0) {}
};

/// 下行调度 Trace 管理器
class dl_scheduler_trace_manager
{
public:
  mutable bool already_access = false;

  explicit dl_scheduler_trace_manager(const std::string& trace_file);

  /// 获取指定时隙的 trace 样本（顺序循环访问，基于时间推进索引）
  std::optional<dl_scheduler_trace_sample> get_trace_sample(slot_point slot) const;

  bool   is_valid() const { return !trace_samples_.empty(); }
  size_t size() const { return trace_samples_.size(); }
  void   set_enabled(bool enable) { enabled_ = enable; }
  bool   is_enabled() const { return enabled_; }
  void   set_start_slot(unsigned start_slot) { start_slot_ = start_slot; }

  std::optional<dl_scheduler_trace_sample> get_sample_by_index(size_t index) const;

private:
  void load_trace_file(const std::string& filename);
  bool parse_trace_line(const std::string& line, dl_scheduler_trace_sample& sample);

  std::vector<dl_scheduler_trace_sample> trace_samples_;
  std::unordered_map<unsigned, size_t>   slot_to_index_;

  bool                              enabled_       = true;
  mutable size_t                    current_index_ = 0;
  mutable std::optional<slot_point> last_access_slot_;
  unsigned                          start_slot_ = 1;
};

} // namespace srsran
