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

#include "gnb_appconfig_translators.h"
#include "apps/services/worker_manager/worker_manager_config.h"
#include "apps/units/flexible_o_du/o_du_high/du_high/du_high_config.h"
#include "gnb_appconfig.h"

using namespace srsran;
using namespace std::chrono_literals;

void srsran::fill_gnb_worker_manager_config(worker_manager_config& config, const gnb_appconfig& app_cfg)
{
  srsran_assert(config.cu_up_cfg, "CU-UP worker config does not exist");
  srsran_assert(config.du_hi_cfg, "DU high worker config does not exist");

  config.nof_main_pool_threads     = app_cfg.expert_execution_cfg.threads.main_pool.nof_threads;
  config.main_pool_task_queue_size = app_cfg.expert_execution_cfg.threads.main_pool.task_queue_size;
  config.main_pool_backoff_period =
      std::chrono::microseconds{app_cfg.expert_execution_cfg.threads.main_pool.backoff_period};
  config.main_pool_affinity_cfg = app_cfg.expert_execution_cfg.affinities.main_pool_cpu_cfg;
}

void srsran::fill_du_scheduler_config_from_gnb_config(du_high_unit_config& du_cfg, const gnb_appconfig& gnb_cfg)
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
