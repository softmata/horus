#!/usr/bin/env python3
"""HORUS Benchmark Analysis — Generates plots, LaTeX tables, and summary report.

Usage:
    python3 analyze.py --input-dir results/ --output-dir results/
    python3 analyze.py --help

Reads CSV files produced by benchmark binaries and generates
plots (PDF) and LaTeX tables.
"""

import argparse
import os
import sys
import json

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

# Plot defaults
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.figsize': (7, 4.5),
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'grid.alpha': 0.3,
})


def load_csv(path):
    """Load CSV with pandas, fall back to manual parsing."""
    try:
        import pandas as pd
        return pd.read_csv(path)
    except ImportError:
        # Manual fallback without pandas
        with open(path) as f:
            header = f.readline().strip().split(',')
            rows = []
            for line in f:
                rows.append(dict(zip(header, line.strip().split(','))))
        return rows


def plot_latency_cdf(input_dir, output_dir):
    """Plot latency CDF from research_latency CSV output."""
    csv_path = os.path.join(input_dir, 'latency.csv')
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} not found")
        return

    import pandas as pd
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots()
    for (backend, size), group in df.groupby(['backend', 'msg_size_bytes']):
        data = group['latency_ns'].sort_values().values
        # Subsample for large datasets (>100K points)
        if len(data) > 100_000:
            idx = np.linspace(0, len(data)-1, 10_000, dtype=int)
            data = data[idx]
        cdf = np.arange(1, len(data)+1) / len(data)
        label = f"{backend} {size}B"
        ax.plot(data, cdf, label=label, linewidth=1.2)

    ax.set_xlabel('Latency (ns)')
    ax.set_ylabel('CDF')
    ax.set_xscale('log')
    ax.set_title('IPC Latency Distribution (All Backends × Sizes)')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True)
    out = os.path.join(output_dir, 'latency_cdf.pdf')
    fig.savefig(out)
    plt.close(fig)
    print(f"  [plot] {out}")


def plot_throughput(input_dir, output_dir):
    """Plot per-second throughput timeseries."""
    csv_path = os.path.join(input_dir, 'throughput.csv')
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} not found")
        return

    import pandas as pd
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots()
    ax.plot(df['second'], df['msgs_per_sec'] / 1e6, 'b-', linewidth=1.5)
    ax.fill_between(df['second'], df['msgs_per_sec'] / 1e6, alpha=0.15)
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Throughput (M msgs/sec)')
    ax.set_title('Sustained Throughput Over Time')
    ax.grid(True)
    out = os.path.join(output_dir, 'throughput_timeseries.pdf')
    fig.savefig(out)
    plt.close(fig)
    print(f"  [plot] {out}")


def plot_jitter(input_dir, output_dir):
    """Plot jitter histograms (RT tick + IPC under contention)."""
    csv_path = os.path.join(input_dir, 'jitter.csv')
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} not found")
        return

    import pandas as pd
    df = pd.read_csv(csv_path)

    for test_name in df['test'].unique():
        subset = df[df['test'] == test_name]['value_ns'].values
        fig, ax = plt.subplots()

        if 'tick' in test_name:
            # RT tick intervals in microseconds
            values = subset / 1000.0
            ax.hist(values, bins=100, edgecolor='black', linewidth=0.5, color='steelblue')
            ax.set_xlabel('Tick Interval (μs)')
            ax.axvline(x=1000, color='red', linestyle='--', label='Expected (1000μs)', linewidth=1)
            ax.legend()
        else:
            # IPC latency in nanoseconds
            values = subset
            ax.hist(values, bins=100, edgecolor='black', linewidth=0.5, color='coral')
            ax.set_xlabel('Latency (ns)')

        ax.set_ylabel('Count')
        ax.set_title(f'Jitter Histogram: {test_name}')
        ax.grid(True, alpha=0.3)
        safe_name = test_name.replace('/', '_').replace(' ', '_')
        out = os.path.join(output_dir, f'jitter_{safe_name}.pdf')
        fig.savefig(out)
        plt.close(fig)
        print(f"  [plot] {out}")


def plot_scalability(input_dir, output_dir):
    """Plot node and topic scaling curves."""
    csv_path = os.path.join(input_dir, 'scale.csv')
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} not found")
        return

    import pandas as pd
    df = pd.read_csv(csv_path)

    # Node scaling
    node_data = df[(df['test'] == 'node_scaling') & (df['metric'] == 'total_tick_us')]
    if not node_data.empty:
        fig, ax = plt.subplots()
        ax.plot(node_data['count'], node_data['value'], 'bo-', linewidth=1.5, markersize=5)
        ax.set_xlabel('Node Count')
        ax.set_ylabel('Tick Duration (μs)')
        ax.set_title('Scheduler Overhead vs Node Count')
        ax.grid(True)
        out = os.path.join(output_dir, 'scalability_nodes.pdf')
        fig.savefig(out)
        plt.close(fig)
        print(f"  [plot] {out}")

    # Topic scaling
    topic_data = df[(df['test'] == 'topic_scaling') & (df['metric'] == 'p50_ns')]
    if not topic_data.empty:
        fig, ax = plt.subplots()
        ax.plot(topic_data['count'], topic_data['value'], 'rs-', linewidth=1.5, markersize=5)
        ax.set_xlabel('Topic Count')
        ax.set_ylabel('Send+Recv Latency p50 (ns)')
        ax.set_title('IPC Latency vs Topic Count')
        ax.set_xscale('log')
        ax.grid(True)
        out = os.path.join(output_dir, 'scalability_topics.pdf')
        fig.savefig(out)
        plt.close(fig)
        print(f"  [plot] {out}")


def plot_comparison(input_dir, output_dir):
    """Plot competitor comparison bar chart."""
    csv_path = os.path.join(input_dir, 'comparison.csv')
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} not found")
        return

    import pandas as pd
    df = pd.read_csv(csv_path)

    sizes = df['msg_size_bytes'].unique()
    competitors = df['competitor'].unique()

    fig, ax = plt.subplots()
    x = np.arange(len(sizes))
    width = 0.8 / len(competitors)
    colors = ['#2196F3', '#FF5722', '#4CAF50', '#9C27B0']

    for i, comp in enumerate(competitors):
        subset = df[df['competitor'] == comp]
        p50s = [subset[subset['msg_size_bytes'] == s]['p50_ns'].values[0] for s in sizes]
        ax.bar(x + i * width, p50s, width, label=comp, color=colors[i % len(colors)])

    ax.set_xlabel('Message Size (bytes)')
    ax.set_ylabel('Latency p50 (ns)')
    ax.set_title('Competitor Comparison — p50 Latency')
    ax.set_xticks(x + width * (len(competitors) - 1) / 2)
    ax.set_xticklabels([f"{s}B" for s in sizes])
    ax.set_yscale('log')
    ax.legend()
    ax.grid(True, axis='y')
    out = os.path.join(output_dir, 'comparison_bar.pdf')
    fig.savefig(out)
    plt.close(fig)
    print(f"  [plot] {out}")


def generate_latex_table(input_dir, output_dir):
    """Generate LaTeX table from comparison CSV."""
    csv_path = os.path.join(input_dir, 'comparison.csv')
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} not found")
        return

    import pandas as pd
    df = pd.read_csv(csv_path)

    lines = []
    lines.append(r'\begin{tabular}{lrrrrr}')
    lines.append(r'\toprule')
    lines.append(r'Middleware & Size & p50 (ns) & p99 (ns) & p999 (ns) & Samples \\')
    lines.append(r'\midrule')

    for _, row in df.iterrows():
        lines.append(
            f"{row['competitor']} & {row['msg_size_bytes']}B & "
            f"{int(row['p50_ns'])} & {int(row['p99_ns'])} & "
            f"{int(row['p999_ns'])} & {int(row['samples']):,} \\\\"
        )

    lines.append(r'\bottomrule')
    lines.append(r'\end{tabular}')

    out = os.path.join(output_dir, 'comparison_table.tex')
    with open(out, 'w') as f:
        f.write('\n'.join(lines))
    print(f"  [table] {out}")


def generate_report(input_dir, output_dir):
    """Generate summary report with all results."""
    report_lines = ["HORUS Benchmark Report", "=" * 40, ""]

    # Load baselines JSON if available
    baselines = os.path.join(input_dir, 'baselines.json')
    if os.path.exists(baselines):
        with open(baselines) as f:
            data = json.load(f)
        report_lines.append(f"Platform: {data.get('platform', {}).get('cpu', 'unknown')}")
        report_lines.append(f"Kernel: {data.get('platform', {}).get('kernel', 'unknown')}")
        report_lines.append("")
        report_lines.append("Raw Baselines:")
        for r in data.get('results', []):
            report_lines.append(f"  {r['test']:20s} {r['label']:8s} p50={r['p50_ns']}ns p99={r['p99_ns']}ns")
        report_lines.append("")

    # List generated files
    report_lines.append("Generated files:")
    for f in sorted(os.listdir(output_dir)):
        size = os.path.getsize(os.path.join(output_dir, f))
        report_lines.append(f"  {f:40s} {size:>10,} bytes")

    out = os.path.join(output_dir, 'report.txt')
    with open(out, 'w') as f:
        f.write('\n'.join(report_lines) + '\n')
    print(f"  [report] {out}")


def main():
    parser = argparse.ArgumentParser(description='HORUS Benchmark Analysis')
    parser.add_argument('--input-dir', default='results', help='Directory with CSV/JSON benchmark output')
    parser.add_argument('--output-dir', default=None, help='Directory for plots/tables (default: same as input)')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_dir = args.output_dir or input_dir
    os.makedirs(output_dir, exist_ok=True)

    print(f"Input:  {os.path.abspath(input_dir)}")
    print(f"Output: {os.path.abspath(output_dir)}")
    print()

    print("Generating plots...")
    plot_latency_cdf(input_dir, output_dir)
    plot_throughput(input_dir, output_dir)
    plot_jitter(input_dir, output_dir)
    plot_scalability(input_dir, output_dir)
    plot_comparison(input_dir, output_dir)

    print("\nGenerating tables...")
    generate_latex_table(input_dir, output_dir)

    print("\nGenerating report...")
    generate_report(input_dir, output_dir)

    print("\nDone.")


if __name__ == '__main__':
    main()
