#!/usr/bin/env python3
"""
generate_comparison.py
======================
Generates a cross-planner comparison report and charts.
Reads from experiments/benchmarks/<Planner>/run_<NNN>/metrics.json
Generates:
  - experiments/benchmarks/comparison/planner_comparison.csv
  - experiments/benchmarks/comparison/comparison_report.md
  - experiments/benchmarks/comparison/comparison_plots.png
"""

import os
import glob
import json
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    benchmarks_dir = script_dir  # We are already in experiments/benchmarks
    out_dir = os.path.join(benchmarks_dir, "comparison")
    os.makedirs(out_dir, exist_ok=True)

    # Planners and their folders
    planners = ["astar", "dijkstra", "rrt"]
    folder_map = {"astar": "A_star", "dijkstra": "Dijkstra", "rrt": "RRT"}

    all_runs = []

    # Read all metrics.json files
    for planner in planners:
        folder = folder_map.get(planner)
        planner_dir = os.path.join(benchmarks_dir, folder)
        json_files = glob.glob(os.path.join(planner_dir, "run_*", "metrics.json"))
        
        for jf in json_files:
            try:
                with open(jf, "r") as f:
                    data = json.load(f)
                
                # Extract relevant fields
                run_data = {
                    "planner": planner,
                    "run_folder": os.path.basename(os.path.dirname(jf)),
                    "success": data.get("reached_target", False),
                    "planning_time_ms": data.get("planner_compute_time_ms", 0.0),
                    "mission_time_s": data.get("travel_time_s", 0.0),
                    "path_length_planned_m": data.get("planner_path_length_m", 0.0),
                    "total_distance_travelled_m": data.get("distance_travelled_m", 0.0),
                    "replan_count": data.get("replan_count", 0),
                    "collision_count": data.get("proximity_events", 0),
                    "near_miss_count": data.get("near_miss_count", 0),
                    "min_obstacle_distance_m": data.get("min_obstacle_distance_m", -1.0),
                    "average_speed_m_s": data.get("average_speed_m_s", 0.0),
                    "max_speed_m_s": data.get("max_speed_m_s", 0.0),
                    "nodes_explored": data.get("planner_nodes_explored", 0),
                    "smoothness_score": data.get("planner_smoothness_score", 1.0)
                }
                all_runs.append(run_data)
            except Exception as e:
                print(f"[GenerateComparison] Error reading {jf}: {e}")

    if not all_runs:
        print("[GenerateComparison] No benchmark runs found. Nothing to compare.")
        return

    df = pd.DataFrame(all_runs)
    
    # Save raw comparison CSV
    csv_path = os.path.join(out_dir, "planner_comparison.csv")
    df.to_csv(csv_path, index=False)
    print(f"[GenerateComparison] Saved aggregated CSV -> {csv_path}")

    # Calculate averages per planner
    grouped = df.groupby("planner")
    
    # Markdown Report Generation
    md = "# Planner Comparison Report\n\n"
    md += "This report aggregates results from all benchmark runs.\n\n"
    md += "## Averages per Planner\n\n"
    md += "| Planner | Runs | Success % | Plan Time (ms) | Mission Time (s) | Path Length (m) | Near Misses | Replans | Smoothness |\n"
    md += "|---------|------|-----------|----------------|------------------|-----------------|-------------|---------|------------|\n"
    
    for planner in planners:
        if planner not in grouped.groups:
            md += f"| {planner.upper()} | 0 | N/A | N/A | N/A | N/A | N/A | N/A | N/A |\n"
            continue
            
        group = grouped.get_group(planner)
        runs = len(group)
        success_pct = (group["success"].sum() / runs) * 100
        plan_time = group["planning_time_ms"].mean()
        mission_time = group["mission_time_s"].mean()
        path_len = group["total_distance_travelled_m"].mean()
        near_misses = group["near_miss_count"].mean()
        replans = group["replan_count"].mean()
        smoothness = group["smoothness_score"].mean()
        
        md += f"| {planner.upper()} | {runs} | {success_pct:.1f}% | {plan_time:.2f} | {mission_time:.2f} | {path_len:.2f} | {near_misses:.1f} | {replans:.1f} | {smoothness:.3f} |\n"
        
    md += "\n## Detailed Runs\n\n"
    md += "| Planner | Run | Success | Plan Time (ms) | Mission Time (s) | Path Length (m) | Min Obs Dist (m) |\n"
    md += "|---------|-----|---------|----------------|------------------|-----------------|------------------|\n"
    
    for _, row in df.sort_values(by=["planner", "run_folder"]).iterrows():
        md += f"| {row['planner'].upper()} | {row['run_folder']} | {row['success']} | {row['planning_time_ms']:.2f} | {row['mission_time_s']:.2f} | {row['total_distance_travelled_m']:.2f} | {row['min_obstacle_distance_m']:.2f} |\n"
        
    md_path = os.path.join(out_dir, "comparison_report.md")
    with open(md_path, "w") as f:
        f.write(md)
    print(f"[GenerateComparison] Saved comparison report -> {md_path}")
    
    # Plotting
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle("Planner Benchmarking Comparison (Averages)", fontsize=16)
    
    metrics = [
        ("planning_time_ms", "Planning Time (ms)", "#1f77b4"),
        ("mission_time_s", "Mission Time (s)", "#ff7f0e"),
        ("total_distance_travelled_m", "Distance Travelled (m)", "#2ca02c"),
        ("near_miss_count", "Near Misses", "#d62728"),
        ("replan_count", "Replans", "#9467bd"),
        ("smoothness_score", "Smoothness (lower=smoother)", "#8c564b")
    ]
    
    labels = []
    means = {m[0]: [] for m in metrics}
    
    for planner in planners:
        if planner in grouped.groups:
            labels.append(planner.upper())
            group = grouped.get_group(planner)
            for m in metrics:
                means[m[0]].append(group[m[0]].mean())
                
    if labels:
        x = np.arange(len(labels))
        width = 0.5
        
        for idx, (col, title, color) in enumerate(metrics):
            ax = axes[idx // 3, idx % 3]
            vals = means[col]
            bars = ax.bar(x, vals, width, color=color)
            ax.set_title(title)
            ax.set_xticks(x)
            ax.set_xticklabels(labels)
            
            for bar in bars:
                height = bar.get_height()
                ax.annotate(f'{height:.2f}',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3),  
                            textcoords="offset points",
                            ha='center', va='bottom', fontsize=9)
                            
        plt.tight_layout()
        plot_path = os.path.join(out_dir, "comparison_plots.png")
        plt.savefig(plot_path)
        print(f"[GenerateComparison] Saved comparison plots -> {plot_path}")
    else:
        print("[GenerateComparison] Could not generate plots: missing data.")
        
if __name__ == "__main__":
    main()
