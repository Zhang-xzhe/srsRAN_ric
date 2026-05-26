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

### 4. `apps/services/worker_manager/worker_manager_appconfig.h`

新增 `<string>` include，并在文件末尾新增 `scheduler_appconfig` 结构体，再将其加入 `expert_execution_appconfig`：

```cpp
#pragma once

#include "os_sched_affinity_manager.h"
#include <string>   // 新增

// ... 已有结构体 ...

/// Scheduler configuration for expert execution.
struct scheduler_appconfig {
  std::string dl_scheduler_trace_file;
  unsigned    dl_trace_start_slot = 1000;
};

/// Expert configuration of the application.
struct expert_execution_appconfig {
  cpu_affinities_appconfig affinities;
  expert_threads_appconfig threads;
  scheduler_appconfig      scheduler;  // 新增
};
```

---

### 5. `apps/services/worker_manager/worker_manager_cli11_schema.cpp`

在 `configure_cli11_expert_execution_args()` 末尾新增 scheduler 子命令及选项（与 affinities/threads 并列，同属 `expert_execution` 子命令下）：

```cpp
static void configure_cli11_expert_execution_args(CLI::App& app, expert_execution_appconfig& config)
{
  // ... 已有 affinities/threads 部分 ...

  // Scheduler section.
  CLI::App* scheduler_subcmd =
      add_subcommand(app, "scheduler", "Scheduler expert configuration")->configurable();
  scheduler_subcmd->add_option(
      "--dl_scheduler_trace_file",
      config.scheduler.dl_scheduler_trace_file,
      "Path to DL scheduler trace file");
  scheduler_subcmd
      ->add_option(
          "--dl_trace_start_slot",
          config.scheduler.dl_trace_start_slot,
          "Minimum slot number before trace-based scheduling takes effect")
      ->capture_default_str();
}
```

> **注意**：此函数位于 `worker_manager_cli11_schema.cpp`，而非 `gnb_appconfig_cli11_schema.cpp`。`configure_cli11_with_worker_manager_appconfig_schema()` 内部调用它，gNB/DU/CU-UP 等所有 appconfig 均通过该函数统一注册。

---

### 6. `apps/gnb/gnb_appconfig_yaml_writer.cpp`

在 `fill_gnb_appconfig_expert_execution_section()` 末尾新增 scheduler 块写入：

```cpp
{
  YAML::Node scheduler_node             = node["scheduler"];
  scheduler_node["dl_trace_start_slot"] = config.scheduler.dl_trace_start_slot;
  if (!config.scheduler.dl_scheduler_trace_file.empty()) {
    scheduler_node["dl_scheduler_trace_file"] = config.scheduler.dl_scheduler_trace_file;
  }
}
```

---

### 7. `apps/gnb/gnb_appconfig_translators.h`

新增 `du_high_unit_config` 前向声明及函数声明：

```cpp
#pragma once

namespace srsran {

struct gnb_appconfig;
struct worker_manager_config;
struct du_high_unit_config;  // 新增

void fill_gnb_worker_manager_config(worker_manager_config& config, const gnb_appconfig& unit_cfg);

/// 将 expert_execution.scheduler trace 配置传播到 DU 各 cell 的调度器配置中。
void fill_du_scheduler_config_from_gnb_config(du_high_unit_config& du_cfg,
                                              const gnb_appconfig& gnb_cfg);  // 新增

} // namespace srsran
```

---

### 8. `apps/gnb/gnb_appconfig_translators.cpp`

新增 `du_high_config.h` include，并添加 `fill_du_scheduler_config_from_gnb_config` 实现：

```cpp
#include "gnb_appconfig_translators.h"
#include "apps/services/worker_manager/worker_manager_config.h"
#include "apps/units/flexible_o_du/o_du_high/du_high/du_high_config.h"  // 新增
#include "gnb_appconfig.h"

// ... fill_gnb_worker_manager_config 保持不变 ...

void srsran::fill_du_scheduler_config_from_gnb_config(du_high_unit_config& du_cfg,
                                                      const gnb_appconfig& gnb_cfg)
{
  if (gnb_cfg.expert_execution_cfg.scheduler.dl_scheduler_trace_file.empty()) {
    return;
  }
  for (auto& cell_cfg : du_cfg.cells_cfg) {
    cell_cfg.cell.sched_expert_cfg.dl_scheduler_trace_file =
        gnb_cfg.expert_execution_cfg.scheduler.dl_scheduler_trace_file;
    cell_cfg.cell.sched_expert_cfg.dl_trace_start_slot =
        gnb_cfg.expert_execution_cfg.scheduler.dl_trace_start_slot;
  }
}
```

---

### 9. `apps/gnb/gnb.cpp`

在 `app.callback` lambda 开头（`autoderive_slicing_args` 之前）调用新函数：

```cpp
app.callback([&app, &gnb_cfg, &o_du_app_unit, &o_cu_cp_app_unit, &o_cu_up_app_unit]() {
  autoderive_gnb_parameters_after_parsing(app, gnb_cfg);

  cu_cp_unit_config& cu_cp_cfg = o_cu_cp_app_unit->get_o_cu_cp_unit_config().cucp_cfg;

  // 新增：将 expert_execution.scheduler trace 配置传播到 DU cells
  fill_du_scheduler_config_from_gnb_config(
      o_du_app_unit->get_o_du_high_unit_config().du_high_cfg.config, gnb_cfg);

  autoderive_slicing_args(o_du_app_unit->get_o_du_high_unit_config().du_high_cfg.config, cu_cp_cfg);
  // ... 其余代码不变 ...
});
```

---

### 10. `lib/scheduler/scheduler_impl.h`

新增头文件引入和私有成员变量（无需暴露 getter）：

```cpp
#include "srsran/scheduler/scheduler_trace.h"
#include <memory>
// ...

class scheduler_impl final : public mac_scheduler {
  // ... 已有 public 方法 ...

private:
  const scheduler_expert_config expert_params;
  srslog::basic_logger&         logger;

  /// Optional trace manager for trace-based DL scheduling.
  std::unique_ptr<dl_scheduler_trace_manager> dl_trace_mgr;  // 新增

  // ... 其余成员变量 ...
};
```

---

### 11. `lib/scheduler/scheduler_impl.cpp`

新增头文件引入，在构造函数体内（`expert_params` 赋值后）初始化 trace manager，并在 `handle_cell_configuration_request()` 传入指针：

```cpp
#include "scheduler_impl.h"
#include "ue_scheduling/ue_scheduler_impl.h"
#include "srsran/scheduler/scheduler_trace.h"  // 新增
// ...

scheduler_impl::scheduler_impl(const scheduler_config& sched_cfg_) :
  expert_params(sched_cfg_.expert_params),
  logger(srslog::fetch_basic_logger("SCHED")),
  cfg_mng(sched_cfg_, metrics)
{
  // 新增：如果配置了 trace 文件，初始化 trace manager
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

// handle_cell_configuration_request() 中，创建新 group 时传入 trace manager：
groups.emplace(msg.cell_group_index,
    std::make_unique<ue_scheduler_impl>(expert_params.ue, dl_trace_mgr.get()));  // 修改
```

---

### 12. `lib/scheduler/ue_scheduling/ue_scheduler_impl.h`

新增头文件引入，修改构造函数声明，并在私有成员变量中存储 trace manager 指针（用于传递给 `intra_slice_sched`）：

```cpp
#include "srsran/scheduler/scheduler_trace.h"  // 新增
// ...

class ue_scheduler_impl final : public ue_scheduler {
public:
  explicit ue_scheduler_impl(const scheduler_ue_expert_config& expert_cfg_,
                              dl_scheduler_trace_manager*       trace_mgr_ = nullptr);  // 修改
  // ...

private:
  // ...
  const scheduler_ue_expert_config& expert_cfg;
  srslog::basic_logger&             logger;

  /// Optional trace manager for trace-based DL scheduling.
  dl_scheduler_trace_manager* trace_mgr;  // 新增

  // ...
};
```

---

### 13. `lib/scheduler/ue_scheduling/ue_scheduler_impl.cpp`

修改构造函数签名，将 `trace_mgr_` 存入成员变量，并在 `cell_context` 的初始化列表中将其传递给 `intra_slice_sched`：

```cpp
// 构造函数修改：
ue_scheduler_impl::ue_scheduler_impl(const scheduler_ue_expert_config& expert_cfg_,
                                     dl_scheduler_trace_manager*       trace_mgr_) :
  expert_cfg(expert_cfg_),
  logger(srslog::fetch_basic_logger("SCHED")),
  trace_mgr(trace_mgr_),  // 新增
  event_mng(ue_db)
{}

// cell_context 初始化列表中，intra_slice_sched 构造处新增最后一个参数：
ue_scheduler_impl::cell_context::cell_context(...) :
  // ...
  intra_slice_sched(parent.expert_cfg,
                    parent.ue_db,
                    *params.pdcch_sched,
                    *params.uci_alloc,
                    *params.cell_res_alloc,
                    *params.cell_metrics,
                    cell_harqs,
                    srslog::fetch_basic_logger("SCHED"),
                    parent.trace_mgr),  // 新增
  // ...
```

---

### 14. `lib/scheduler/ue_scheduling/intra_slice_scheduler.h`（**Trace 数据链路节点**）

新增头文件引入，扩展构造函数签名，并添加私有成员变量：

```cpp
#include "srsran/scheduler/scheduler_trace.h"  // 新增
// ...

class intra_slice_scheduler {
public:
  intra_slice_scheduler(const scheduler_ue_expert_config& expert_cfg_,
                        ue_repository&                    ues,
                        pdcch_resource_allocator&         pdcch_alloc,
                        uci_allocator&                    uci_alloc,
                        cell_resource_allocator&          cell_alloc,
                        cell_metrics_handler&             cell_metrics_,
                        cell_harq_manager&                cell_harqs_,
                        srslog::basic_logger&             logger_,
                        dl_scheduler_trace_manager*       trace_mgr_ = nullptr);  // 新增参数
  // ...

private:
  // ... 已有成员 ...
  dl_scheduler_trace_manager* trace_mgr;  // 新增
  // ...
};
```

---

### 15. `lib/scheduler/ue_scheduling/intra_slice_scheduler.cpp`（**核心 Trace 应用点**）

构造函数签名与初始化列表中接收并存储 `trace_mgr_`：

```cpp
intra_slice_scheduler::intra_slice_scheduler(
    const scheduler_ue_expert_config& expert_cfg_,
    // ...
    srslog::basic_logger&             logger_,
    dl_scheduler_trace_manager*       trace_mgr_) :  // 新增参数
  expert_cfg(expert_cfg_),
  // ...
  logger(logger_),
  trace_mgr(trace_mgr_),  // 新增
  expected_pdschs_per_slot(...),
  ue_alloc(...)
{}
```

在 `schedule_dl_newtx_candidates()` 的 Stage 1 循环中，**在调用 `ue_alloc.allocate_dl_grant()` 前**用 trace 覆盖 `pending_bytes`：

```cpp
// 原来：
auto result = ue_alloc.allocate_dl_grant(
    ue_newtx_dl_grant_request{*ue_candidate.ue, pdsch_slot, ue_candidate.pending_bytes});

// 替换为：
unsigned effective_pending_bytes = ue_candidate.pending_bytes;
if (trace_mgr != nullptr && trace_mgr->is_enabled()) {
  auto trace_sample = trace_mgr->get_trace_sample(pdcch_slot);
  if (trace_sample.has_value()) {
    effective_pending_bytes = trace_sample->tbs;
  }
}
auto result = ue_alloc.allocate_dl_grant(
    ue_newtx_dl_grant_request{*ue_candidate.ue, pdsch_slot, effective_pending_bytes});
```

> **注意**：trace 读取使用 `pdcch_slot`（已在 `slot_indication()` 中更新），不需要 `dynamic_cast`，实现更简洁直接。

---

### 16. `lib/scheduler/CMakeLists.txt`

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
调度流程（intra_slice_scheduler.cpp → schedule_dl_newtx_candidates）：

  1. 计算 pending_bytes = ue_candidate.pending_bytes（来自 RLC 缓冲区）
  2. 若 trace_mgr 已启用且当前 slot >= start_slot_：
       从 trace 获取 tbs → 覆盖 effective_pending_bytes
  3. 将 effective_pending_bytes 传入 ue_alloc.allocate_dl_grant()
  4. allocate_dl_grant() 调用 compute_newtx_required_mcs_and_prbs()，
     根据 effective_pending_bytes 计算出 MCS 和实际 TBS，分配 RB
  5. Trace 索引根据时隙经过数自动推进，支持循环播放
```

### 数据配置链

```
YAML: expert_execution.scheduler.dl_scheduler_trace_file
    → worker_manager_appconfig.h: expert_execution_appconfig::scheduler
        → gnb_appconfig_translators.cpp: fill_du_scheduler_config_from_gnb_config()
            → du_high_unit_scheduler_expert_config::dl_scheduler_trace_file
                → du_high_config_translators.cpp: generate_scheduler_expert_config()
                    → scheduler_expert_config::dl_scheduler_trace_file
```

### 运行时数据传递链

```
scheduler_expert_config.dl_scheduler_trace_file
    → scheduler_impl（构造时创建 dl_scheduler_trace_manager，存为 dl_trace_mgr）
        → ue_scheduler_impl（构造参数传入，存为成员 trace_mgr）
            → intra_slice_scheduler（构造参数传入，存为成员 trace_mgr）
                → schedule_dl_newtx_candidates()（直接使用 trace_mgr 覆盖 pending_bytes）
```

---

## 六、关键注意事项

1. **`already_access` 标志**：一次性标志，处理首次访问 `get_trace_sample()` 时的边界情况，`mutable` 声明允许在 `const` 方法中修改。

2. **时间驱动索引推进**：`get_trace_sample()` 使用 `slots_elapsed`（经过时隙数）推进索引，而非每次调用 +1，确保多 UE 场景下 trace 播放速度与真实时间对齐。

3. **Trace 注入点在 `intra_slice_scheduler.cpp`**：trace 覆盖发生在 `schedule_dl_newtx_candidates()` 的调用处，直接替换传入 `allocate_dl_grant()` 的 `pending_bytes` 参数，无需 `dynamic_cast`。

4. **`scheduler_appconfig` 的位置**：该结构体位于 `worker_manager_appconfig.h`（而不是 `gnb_appconfig.h`），CLI11 注册在 `worker_manager_cli11_schema.cpp` 的 `configure_cli11_expert_execution_args()` 中，对所有 app（gnb/du/cu 等）全局生效。

5. **配置传递需要两步**：YAML `expert_execution.scheduler` → `gnb_appconfig_translators.cpp:fill_du_scheduler_config_from_gnb_config()` 写入 `du_high_unit_scheduler_expert_config` → `du_high_config_translators.cpp:generate_scheduler_expert_config()` 再写入最终的 `scheduler_expert_config`。`gnb.cpp` 的 callback 中需显式调用 `fill_du_scheduler_config_from_gnb_config()`。

6. **CMakeLists.txt**：务必将 `scheduler_trace.cpp` 加入编译目标，否则会有链接错误。

7. **Trace 循环播放**：`current_index_` 对 `trace_samples_.size()` 取模，trace 自然循环，支持任意长时间仿真。
