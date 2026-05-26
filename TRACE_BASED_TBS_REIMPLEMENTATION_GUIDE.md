# 指令：在 srsRAN gNB 调度器中实现基于 Trace 的 PDSCH TBS 覆盖功能

## 背景与目标

在**标准 srsRAN 项目**（未修改版本）中，从零实现一套完整的 "trace-based PDSCH TBS 覆盖" 机制。该机制允许调度器在每个时隙将 UE 的下行传输字节数（TBS/recommended_nof_bytes）替换为预先录制的 CSV trace 中的值，从而重放真实信道测量场景，用于测试和仿真。

---

## 一、需要新建的文件

### 1. `include/srsran/scheduler/scheduler_trace.h`

```cpp
/*
 * Copyright 2021-2024 Software Radio Systems Limited
 * (标准 srsRAN AGPL v3 头部)
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
  unsigned                  slot_index;   ///< Trace 内的时隙索引
  sch_mcs_index             mcs;          ///< MCS 索引（TBS-only 格式时设为 0）
  unsigned                  tbs;          ///< 传输块大小（字节）
  bool                      needs_retx;   ///< 是否为重传
  unsigned                  retx_count;   ///< 重传次数（0 = 首次）
  std::optional<harq_id_t>  harq_id;      ///< HARQ 进程 ID（可选）

  dl_scheduler_trace_sample() :
    slot_index(0), mcs(0), tbs(0), needs_retx(false), retx_count(0) {}
};

/// 下行调度 Trace 管理器
class dl_scheduler_trace_manager {
public:
  mutable bool already_access = false;

  explicit dl_scheduler_trace_manager(const std::string& trace_file);

  /// 获取指定时隙的 trace 样本（顺序循环访问，基于时间推进索引）
  std::optional<dl_scheduler_trace_sample> get_trace_sample(slot_point slot) const;

  bool   is_valid()   const { return !trace_samples_.empty(); }
  size_t size()       const { return trace_samples_.size(); }
  void   set_enabled(bool enable) { enabled_ = enable; }
  bool   is_enabled() const { return enabled_; }
  void   set_start_slot(unsigned start_slot) { start_slot_ = start_slot; }

  std::optional<dl_scheduler_trace_sample> get_sample_by_index(size_t index) const;

private:
  void load_trace_file(const std::string& filename);
  bool parse_trace_line(const std::string& line, dl_scheduler_trace_sample& sample);

  std::vector<dl_scheduler_trace_sample>    trace_samples_;
  std::unordered_map<unsigned, size_t>      slot_to_index_;

  bool                              enabled_       = true;
  mutable size_t                    current_index_ = 0;
  mutable std::optional<slot_point> last_access_slot_;
  unsigned                          start_slot_    = 1;
};

} // namespace srsran
```

---

### 2. `lib/scheduler/scheduler_trace.cpp`

```cpp
/*
 * Copyright 2021-2024 Software Radio Systems Limited
 * (标准 srsRAN AGPL v3 头部)
 */

#include "srsran/scheduler/scheduler_trace.h"
#include "srsran/srslog/srslog.h"
#include "srsran/ran/slot_point.h"
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace srsran;

dl_scheduler_trace_manager::dl_scheduler_trace_manager(const std::string& trace_file)
{
  if (!trace_file.empty()) {
    load_trace_file(trace_file);
  }
}

void dl_scheduler_trace_manager::load_trace_file(const std::string& filename)
{
  auto& logger = srslog::fetch_basic_logger("SCHED");
  std::ifstream file(filename);
  if (!file.is_open()) {
    logger.warning("Failed to open scheduler trace file '{}'. Trace-based scheduling disabled.", filename);
    enabled_ = false;
    return;
  }

  std::string line;
  size_t line_num = 0, valid_samples = 0;
  while (std::getline(file, line)) {
    ++line_num;
    if (line.empty() || line[0] == '#') continue;
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
    logger.info("Loaded {} scheduler trace samples from '{}' ({} lines processed)",
                valid_samples, filename, line_num);
    enabled_ = true;
  }
}

bool dl_scheduler_trace_manager::parse_trace_line(const std::string& line,
                                                   dl_scheduler_trace_sample& sample)
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
    if (iss.fail()) return false;
    sample.mcs        = sch_mcs_index{0};
    sample.needs_retx = false;
    sample.retx_count = 0;
    sample.harq_id    = std::nullopt;
  } else {
    // 完整格式：slot_index,mcs,tbs_bytes,needs_retx,retx_count[,harq_id]
    unsigned mcs_val;
    iss >> sample.slot_index >> comma >> mcs_val >> comma
        >> sample.tbs >> comma >> needs_retx_int >> comma >> sample.retx_count;
    sample.mcs = sch_mcs_index{static_cast<uint8_t>(mcs_val)};
    if (iss.fail()) return false;
    if (iss >> comma >> harq_id_int && harq_id_int >= 0)
      sample.harq_id = to_harq_id(harq_id_int);
    sample.needs_retx = (needs_retx_int != 0);
    if (sample.mcs > 28) return false;
  }

  if (sample.tbs == 0 || sample.retx_count > 4) return false;
  return true;
}

std::optional<dl_scheduler_trace_sample>
dl_scheduler_trace_manager::get_trace_sample(slot_point slot) const
{
  if (!enabled_ || trace_samples_.empty()) return std::nullopt;

  // 达到 start_slot_ 之前不应用 trace（保护 UE 接入）
  if (slot.to_uint() < start_slot_ && !already_access) {
    already_access = true;
    return std::nullopt;
  }

  // 根据经过的时隙数推进 current_index_（时间驱动）
  if (last_access_slot_.has_value()) {
    unsigned slots_elapsed = slot.to_uint() - last_access_slot_->to_uint();
    current_index_ = (current_index_ + slots_elapsed) % trace_samples_.size();
  }
  last_access_slot_ = slot;

  return trace_samples_[current_index_];
}

std::optional<dl_scheduler_trace_sample>
dl_scheduler_trace_manager::get_sample_by_index(size_t index) const
{
  if (!enabled_ || trace_samples_.empty() || index >= trace_samples_.size())
    return std::nullopt;
  return trace_samples_[index];
}
```

---

## 二、需要修改的文件

### 3. `include/srsran/scheduler/config/scheduler_expert_config.h`

在 `scheduler_expert_config` 结构体末尾新增两个字段（其他字段保持不变）：

```cpp
struct scheduler_expert_config {
  // ... 已有字段保持不变 ...
  bool                           log_broadcast_messages;
  std::chrono::milliseconds      metrics_report_period;

  // 新增：
  /// Path to DL scheduler trace file (optional)
  std::string  dl_scheduler_trace_file;
  /// Minimum slot number before trace-based scheduling takes effect (default: 1000)
  unsigned     dl_trace_start_slot = 1000;
};
```

---

### 4. `apps/gnb/gnb_appconfig.h`

新增调度器应用配置结构体 `scheduler_appconfig`，并将其加入 `expert_execution_appconfig`：

```cpp
/// Scheduler configuration for expert execution.
struct scheduler_appconfig {
  std::string dl_scheduler_trace_file;
  unsigned    dl_trace_start_slot = 1000;
};

struct expert_execution_appconfig {
  // ... 已有字段 ...
  scheduler_appconfig scheduler;  // 新增
};
```

---

### 5. `apps/gnb/gnb_appconfig_cli11_schema.cpp`

在 `configure_cli11_expert_execution_args()` 内新增 scheduler 子命令及选项：

```cpp
// 新增 scheduler 子命令
CLI::App* scheduler_subcmd =
    add_subcommand(app, "scheduler", "Scheduler expert configuration")->configurable();
scheduler_subcmd->add_option(
    "--dl_scheduler_trace_file",
    config.scheduler.dl_scheduler_trace_file,
    "Path to DL scheduler trace file");
scheduler_subcmd->add_option(
    "--dl_trace_start_slot",
    config.scheduler.dl_trace_start_slot,
    "Minimum slot number before trace-based scheduling takes effect")
    ->capture_default_str();
```

---

### 6. `apps/gnb/gnb_appconfig_yaml_writer.cpp`

在 scheduler 部分的 YAML 写入函数中新增：

```cpp
if (!config.scheduler.dl_scheduler_trace_file.empty()) {
  scheduler_node["dl_scheduler_trace_file"] = config.scheduler.dl_scheduler_trace_file;
}
scheduler_node["dl_trace_start_slot"] = config.scheduler.dl_trace_start_slot;
```

---

### 7. `apps/gnb/gnb_appconfig_translators.cpp`

新增（或修改现有的）配置传递函数，将 gnb appconfig 中的 trace 配置传递给 DU：

```cpp
void srsran::fill_du_scheduler_config_from_gnb_config(du_high_unit_config& du_cfg,
                                                       const gnb_appconfig& gnb_cfg)
{
  for (auto& cell_cfg : du_cfg.cells_cfg) {
    cell_cfg.cell.sched_expert_cfg.dl_scheduler_trace_file =
        gnb_cfg.expert_execution_cfg.scheduler.dl_scheduler_trace_file;
    cell_cfg.cell.sched_expert_cfg.dl_trace_start_slot =
        gnb_cfg.expert_execution_cfg.scheduler.dl_trace_start_slot;
  }
}
```

该函数声明需要加入对应的头文件，并在合适的初始化流程中调用。

---

### 8. `lib/scheduler/scheduler_impl.h`

新增头文件引入和成员变量：

```cpp
#include "srsran/scheduler/scheduler_trace.h"
// ...

class scheduler_impl : public scheduler {
public:
  // ...
  dl_scheduler_trace_manager*       get_dl_trace_manager()       { return dl_trace_mgr.get(); }
  const dl_scheduler_trace_manager* get_dl_trace_manager() const { return dl_trace_mgr.get(); }

private:
  // ...
  std::unique_ptr<dl_scheduler_trace_manager> dl_trace_mgr;
};
```

---

### 9. `lib/scheduler/scheduler_impl.cpp`

在构造函数体内（`expert_params` 赋值后）初始化 trace manager：

```cpp
scheduler_impl::scheduler_impl(const scheduler_config& sched_cfg_) :
  expert_params(sched_cfg_.expert_params),
  // ...
{
  // 如果配置了 trace 文件，初始化 trace manager
  if (!expert_params.dl_scheduler_trace_file.empty()) {
    dl_trace_mgr = std::make_unique<dl_scheduler_trace_manager>(
        expert_params.dl_scheduler_trace_file);
    if (dl_trace_mgr->is_valid()) {
      dl_trace_mgr->set_start_slot(expert_params.dl_trace_start_slot);
      logger.info("DL scheduler trace-based mode enabled with {} samples (starts at slot {})",
                  dl_trace_mgr->size(), expert_params.dl_trace_start_slot);
    } else {
      logger.warning("DL scheduler trace file provided but invalid. Running without trace.");
      dl_trace_mgr.reset();
    }
  }
}
```

在 `handle_cell_configuration_request()` 中，创建 `ue_scheduler_impl` 时传入 trace manager：

```cpp
// 当创建新 group 时：
groups.emplace(msg.cell_group_index,
    std::make_unique<ue_scheduler_impl>(expert_params.ue, dl_trace_mgr.get()));
```

---

### 10. `lib/scheduler/ue_scheduling/ue_scheduler_impl.h`

修改构造函数声明，接受 trace manager 指针：

```cpp
#include "srsran/scheduler/scheduler_trace.h"
// ...

class ue_scheduler_impl final : public ue_scheduler {
public:
  explicit ue_scheduler_impl(const scheduler_ue_expert_config& expert_cfg_,
                              dl_scheduler_trace_manager*       trace_mgr_ = nullptr);
  // ...
};
```

---

### 11. `lib/scheduler/ue_scheduling/ue_scheduler_impl.cpp`

修改构造函数，将 trace manager 传递给 `ue_cell_grid_allocator`：

```cpp
ue_scheduler_impl::ue_scheduler_impl(const scheduler_ue_expert_config& expert_cfg_,
                                     dl_scheduler_trace_manager*       trace_mgr_) :
  expert_cfg(expert_cfg_),
  ue_alloc(expert_cfg, ue_db, srslog::fetch_basic_logger("SCHED"), trace_mgr_),
  // ...
{}
```

---

### 12. `lib/scheduler/ue_scheduling/ue_cell_grid_allocator.h`

修改构造函数声明，接受并存储 trace manager 指针；暴露 getter；在
`dl_slice_ue_cell_grid_allocator` 上也暴露同名 getter：

```cpp
#include "srsran/scheduler/scheduler_trace.h"
// ...

class ue_cell_grid_allocator {
public:
  ue_cell_grid_allocator(const scheduler_ue_expert_config& expert_cfg_,
                         ue_repository&                    ues_,
                         srslog::basic_logger&             logger_,
                         dl_scheduler_trace_manager*       trace_mgr_ = nullptr);
  // ...
  dl_scheduler_trace_manager* get_trace_manager() const { return trace_mgr; }

private:
  // ...
  dl_scheduler_trace_manager* trace_mgr;
};

// dl_slice_ue_cell_grid_allocator 也需暴露 getter（转发到底层分配器）：
class dl_slice_ue_cell_grid_allocator : public ue_pdsch_allocator {
public:
  // ...
  dl_scheduler_trace_manager* get_trace_manager() const
  {
    return alloc.get_trace_manager();
  }
private:
  ue_cell_grid_allocator& alloc;
  // ...
};
```

---

### 13. `lib/scheduler/ue_scheduling/ue_cell_grid_allocator.cpp`

修改构造函数，存储 `trace_mgr_`：

```cpp
ue_cell_grid_allocator::ue_cell_grid_allocator(
    const scheduler_ue_expert_config& expert_cfg_,
    ue_repository&                    ues_,
    srslog::basic_logger&             logger_,
    dl_scheduler_trace_manager*       trace_mgr_) :
  expert_cfg(expert_cfg_), ues(ues_), logger(logger_), trace_mgr(trace_mgr_)
{}
```

> **注意**：`ue_cell_grid_allocator.cpp` 中有一段注释掉的 TBS 限制代码（约第 370 行），是另一条失效的代码路径，**不需要启用**。真正生效的路径在下一步的 `scheduler_time_pf.cpp`。

---

### 14. `lib/scheduler/policy/scheduler_time_pf.cpp`（**核心 Trace 应用点**）

需要引入头文件：

```cpp
#include "srsran/scheduler/scheduler_trace.h"
// 确保 dl_slice_ue_cell_grid_allocator 的定义可见
#include "../ue_scheduling/ue_cell_grid_allocator.h"
```

在 `try_dl_alloc()` 函数中，当分配 new TX grant 时，**在设置 `grant.recommended_nof_bytes` 之后、调用 `pdsch_alloc.allocate_dl_grant()` 之前**，插入如下逻辑：

```cpp
if (ctxt.has_empty_dl_harq) {
  grant.h_id                  = INVALID_HARQ_ID;
  grant.recommended_nof_bytes = ues[ctxt.ue_index].pending_dl_newtx_bytes();

  // *** 新增：用 trace 覆盖 recommended_nof_bytes ***
  auto* slice_alloc = dynamic_cast<dl_slice_ue_cell_grid_allocator*>(&pdsch_alloc);
  if (slice_alloc != nullptr) {
    auto* trace_mgr = slice_alloc->get_trace_manager();
    if (trace_mgr != nullptr && trace_mgr->is_enabled()) {
      slot_point current_slot = res_grid.get_pdcch_slot(ctxt.cell_index);
      std::optional<dl_scheduler_trace_sample> trace_sample =
          trace_mgr->get_trace_sample(current_slot);
      if (trace_sample.has_value()) {
        grant.recommended_nof_bytes = trace_sample->tbs;
      }
    }
  }
  // *** 新增结束 ***

  grant.max_nof_rbs = max_rbs;
  alloc_result      = pdsch_alloc.allocate_dl_grant(grant);
  if (alloc_result.status == alloc_status::success) {
    ctxt.has_empty_dl_harq = false;
  }
  return alloc_result;
}
```

---

### 15. `lib/scheduler/CMakeLists.txt`

在 `add_library(srsran_sched ...)` 或等效目标的源文件列表中新增：

```cmake
scheduler_trace.cpp
```

---

## 三、Trace 文件格式

### 格式 1：TBS-only（简洁格式）

```csv
# slot_index,tbs_bytes
0,309
1,63
2,84
```

### 格式 2：完整格式

```csv
# slot_index,mcs,tbs_bytes,needs_retx,retx_count,harq_id
0,16,5376,0,0,0
1,16,5376,0,0,1
2,12,3752,1,1,0
```

### 字段说明

| 字段          | 类型   | 范围     | 说明                          |
|---------------|--------|----------|-------------------------------|
| `slot_index`  | uint   | 0-10239  | 时隙索引                      |
| `mcs`         | uint   | 0-28     | MCS 索引（TBS-only 时设为 0） |
| `tbs_bytes`   | uint   | >0       | 传输块大小（字节）            |
| `needs_retx`  | bool   | 0/1      | 是否需要重传                  |
| `retx_count`  | uint   | 0-4      | 重传次数（0 = 首次传输）      |
| `harq_id`     | uint   | 0-15     | HARQ 进程 ID（可省略）        |

以 `#` 开头的行为注释，空行忽略。

---

## 四、gNB YAML 配置示例

```yaml
expert_execution:
  scheduler:
    dl_scheduler_trace_file: "configs/l4span/my_trace.csv"
    dl_trace_start_slot: 1000   # 前 1000 个时隙不应用 trace，确保 UE 接入完成
```

### 推荐 start_slot 设置

| 场景         | dl_trace_start_slot |
|--------------|---------------------|
| 最小延迟     | 500                 |
| 推荐（默认） | 1000                |
| 保守延迟     | 2000                |

---

## 五、工作原理

```
调度流程（scheduler_time_pf.cpp → try_dl_alloc）：

  1. 计算 recommended_nof_bytes = pending_dl_newtx_bytes()
  2. 若 trace_mgr 已启用且当前 slot >= start_slot_：
       从 trace 获取 tbs → 覆盖 recommended_nof_bytes
  3. 将 recommended_nof_bytes 传入 allocate_dl_grant()
  4. allocate_dl_grant() 调用 required_dl_prbs() 及 compute_dl_mcs_tbs()，
     根据 recommended_nof_bytes 计算出 MCS 和实际 TBS，分配 RB
  5. Trace 索引根据时隙经过数自动推进，支持循环播放
```

### 数据传递链

```
scheduler_expert_config.dl_scheduler_trace_file
    → scheduler_impl（构造时创建 dl_scheduler_trace_manager）
        → ue_scheduler_impl（构造参数传入）
            → ue_cell_grid_allocator（构造参数传入，存储为成员 trace_mgr）
                → dl_slice_ue_cell_grid_allocator::get_trace_manager()
                    → scheduler_time_pf::try_dl_alloc()（通过 dynamic_cast 获取并使用）
```

---

## 六、关键注意事项

1. **`already_access` 标志**：一次性标志，处理首次访问 `get_trace_sample()` 时的边界情况，`mutable` 声明允许在 `const` 方法中修改。

2. **时间驱动索引推进**：`get_trace_sample()` 使用 `slots_elapsed`（经过时隙数）推进索引，而非每次调用 +1，确保多 UE 场景下 trace 播放速度与真实时间对齐。

3. **`dynamic_cast` 的必要性**：调度策略（`scheduler_time_pf`）通过 `ue_pdsch_allocator` 抽象接口访问分配器，需要 `dynamic_cast` 到 `dl_slice_ue_cell_grid_allocator` 才能取回 trace manager。

4. **注释掉的 TBS 限制代码**：`ue_cell_grid_allocator.cpp` 约第 370 行有另一条 trace 代码路径（注释状态），**不应启用**，真正生效的入口仅为 `scheduler_time_pf.cpp`。

5. **CMakeLists.txt**：务必将 `scheduler_trace.cpp` 加入编译目标，否则会有链接错误。

6. **Trace 循环播放**：`current_index_` 对 `trace_samples_.size()` 取模，trace 自然循环，支持任意长时间仿真。
