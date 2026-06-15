#!/usr/bin/env python3
"""
从 srsRAN RLC 日志中解析 queued_bytes 并画图
横轴以毫秒为单位（相对于第一个数据点的时间差）

支持日志格式:
    2026-06-14T04:58:00.849096 [RLC     ] [D] du=0 ue=0 DRB1 DL: Reading SDU from sdu_queue. queued_sdus=307 queued_bytes=316625

用法:
    python3 plot_queued_bytes.py rlc.log
    python3 plot_queued_bytes.py rlc.log -o queued_bytes.png
    python3 plot_queued_bytes.py rlc.log --per-ue -o output.png
"""

import re
import sys
import argparse
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt


def parse_rlc_log(filepath):
    """
    解析 RLC 日志，提取时间戳和 queued_bytes
    """
    data = []
    # 匹配时间戳和 queued_bytes
    # 示例: 2026-06-14T04:58:00.849096 [RLC     ] [D] du=0 ue=0 DRB1 DL: ... queued_bytes=316625
    pattern = re.compile(
        r'^(?P<timestamp>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d+)'  # 时间戳
        r'\s+\[RLC\s*\]\s+\[[DIEWFT]\]'  # [RLC     ] [D]
        r'\s+du=\d+\s+ue=(?P<ue>\d+)'  # du=0 ue=0
        r'\s+\S+\s+DL:'  # DRB1 DL:
        r'.*?'  # 任意内容
        r'queued_bytes=(?P<bytes>\d+)',  # queued_bytes=316625
        re.IGNORECASE
    )

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            match = pattern.search(line)
            if match:
                ts_str = match.group('timestamp')
                ue = int(match.group('ue'))
                val = int(match.group('bytes'))

                try:
                    ts = datetime.fromisoformat(ts_str)
                except ValueError:
                    continue

                data.append((ts, ue, val))

    return data


def plot_queued_bytes(data, title="RLC Queued Bytes", output_path=None, per_ue=False):
    """
    绘制 queued_bytes 时间序列图
    横轴以毫秒为单位（相对于第一个数据点的时间差）

    Args:
        data: list of (datetime_timestamp, ue, bytes)
        per_ue: 是否按 UE 分开绘图
    """
    if not data:
        print("未找到任何 queued_bytes 数据！请检查日志格式。")
        return

    print(f"共解析到 {len(data)} 条 queued_bytes 记录")

    # 计算相对于第一个数据点的时间差（毫秒）
    base_time = data[0][0]
    time_ms = [(ts - base_time).total_seconds() * 1000.0 for ts, _, _ in data]
    total_duration_ms = time_ms[-1] - time_ms[0]
    print(f"时间跨度: {total_duration_ms:.1f} ms")

    if per_ue:
        # 按 UE 分组
        ue_data = {}
        for ts, ue, val in data:
            if ue not in ue_data:
                ue_data[ue] = []
            ue_data[ue].append((ts, val))

        n_ue = len(ue_data)
        fig, axes = plt.subplots(n_ue, 1, figsize=(14, 4 * n_ue), sharex=True)
        if n_ue == 1:
            axes = [axes]

        for ax, (ue, ue_records) in zip(axes, sorted(ue_data.items())):
            # 计算该 UE 的时间差（毫秒）
            ue_base_time = ue_records[0][0]
            ue_time_ms = [(t - ue_base_time).total_seconds() * 1000.0 for t, _ in ue_records]
            values = [v for _, v in ue_records]

            ax.plot(ue_time_ms, values, marker='o', markersize=2, linestyle='-', linewidth=1, alpha=0.8, label=f'UE {ue}')
            ax.set_title(f'UE {ue} - {title}', fontsize=12)
            ax.set_ylabel('Queued Bytes', fontsize=10)
            ax.grid(True, alpha=0.3)

            # 统计信息
            avg_val = sum(values) / len(values)
            max_val = max(values)
            min_val = min(values)
            ax.axhline(y=avg_val, color='r', linestyle='--', alpha=0.5, label=f'Avg: {avg_val:.0f}')
            ax.axhline(y=max_val, color='g', linestyle=':', alpha=0.5, label=f'Max: {max_val}')
            ax.axhline(y=min_val, color='b', linestyle=':', alpha=0.5, label=f'Min: {min_val}')
            ax.legend()

        for ax in axes:
            ax.set_xlabel('Time (ms)', fontsize=10)

    else:
        # 所有 UE 画在一起
        ue_data = {}
        for ts, ue, val in data:
            if ue not in ue_data:
                ue_data[ue] = []
            ue_data[ue].append((ts, val))

        plt.figure(figsize=(14, 6))
        for ue, ue_records in sorted(ue_data.items()):
            # 使用全局 base_time 计算时间差
            ue_time_ms = [(t - base_time).total_seconds() * 1000.0 for t, _ in ue_records]
            values = [v for _, v in ue_records]
            plt.plot(ue_time_ms, values, marker='o', markersize=2, linestyle='-', linewidth=1, alpha=0.8, label=f'UE {ue}')

        plt.title(title, fontsize=14)
        plt.xlabel('Time (ms)', fontsize=12)
        plt.ylabel('Queued Bytes', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.legend()

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"图片已保存到: {output_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='从 srsRAN RLC 日志中绘制 queued_bytes（横轴单位：ms）')
    parser.add_argument('input', help='输入日志文件路径')
    parser.add_argument('-o', '--output', help='输出图片路径（默认直接显示）')
    parser.add_argument('--title', default='RLC Queued Bytes', help='图表标题')
    parser.add_argument('--per-ue', action='store_true', help='按 UE 分开绘制子图')
    args = parser.parse_args()

    filepath = Path(args.input)
    if not filepath.exists():
        print(f"错误: 文件不存在 {filepath}")
        sys.exit(1)

    print(f"正在解析日志文件: {filepath}")
    data = parse_rlc_log(filepath)

    if data:
        base_time = data[0][0]
        end_time = data[-1][0]
        duration_ms = (end_time - base_time).total_seconds() * 1000.0
        print(f"时间范围: {base_time} -> {end_time}")
        print(f"时间跨度: {duration_ms:.1f} ms")
        all_values = [v for _, _, v in data]
        print(f"数值范围: {min(all_values)} -> {max(all_values)} bytes")
        ues = sorted(set(ue for _, ue, _ in data))
        print(f"UE 列表: {ues}")

    plot_queued_bytes(data, title=args.title, output_path=args.output, per_ue=args.per_ue)


if __name__ == '__main__':
    main()
