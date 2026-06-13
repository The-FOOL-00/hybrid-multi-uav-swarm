import os
import glob
import json
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def generate_plots():
    # Paths
    here = os.path.dirname(os.path.abspath(__file__))
    global_csv_path = os.path.join(here, "baseline_runs.csv")
    
    # 1. Generate Comparison Plots if CSV exists
    if os.path.isfile(global_csv_path):
        print(f"[Plots] Loading global runs database: {global_csv_path}")
        df = pd.read_csv(global_csv_path)
        
        # We limit comparison to the last 10 runs to keep the plot readable
        df_plot = df.tail(10)
        
        # Create a figure with 2x3 grid of subplots for different metrics
        fig, axes = plt.subplots(2, 3, figsize=(16, 10))
        fig.suptitle("Single Drone Baseline Metrics Comparison (Last 10 Runs)", fontsize=16, fontweight='bold', color='#1a1a1a')
        
        metrics_to_plot = [
            ("travel_time_s", "Travel Time (s)", "#1f77b4"),
            ("distance_travelled_m", "Path Length (m)", "#ff7f0e"),
            ("average_speed_m_s", "Avg Speed (m/s)", "#2ca02c"),
            ("proximity_events", "Proximity Warnings", "#d62728"),
            ("near_misses", "Near Misses (<=4.0m)", "#9467bd"),
            ("cumulative_coverage_percent", "Cumulative Coverage %", "#8c564b")
        ]
        
        # Use short run labels like "Run #1"
        run_labels = [f"Run {i+1}" for i in range(len(df_plot))]
        if len(df_plot) > 0:
            # Short timestamp label fallback
            timestamps = df_plot["timestamp"].astype(str).tolist()
            run_labels = [ts[-6:] for ts in timestamps]  # last 6 chars of timestamp (HHMMSS)
            
        for idx, (col, title, color) in enumerate(metrics_to_plot):
            ax = axes[idx // 3, idx % 3]
            if col in df_plot.columns:
                values = df_plot[col].tolist()
                bars = ax.bar(run_labels, values, color=color, alpha=0.85, edgecolor='none', width=0.6)
                ax.set_title(title, fontsize=12, fontweight='semibold')
                ax.grid(axis='y', linestyle='--', alpha=0.5)
                # Value labels on top of bars
                for bar in bars:
                    height = bar.get_height()
                    ax.annotate(f'{height:.1f}' if isinstance(height, float) else f'{height}',
                                xy=(bar.get_x() + bar.get_width() / 2, height),
                                xytext=(0, 3),  # 3 points vertical offset
                                textcoords="offset points",
                                ha='center', va='bottom', fontsize=9)
            else:
                ax.text(0.5, 0.5, "Metric missing", ha='center', va='center')
                ax.set_title(title)
        
        plt.tight_layout(rect=[0, 0, 1, 0.95])
        comp_plot_path = os.path.join(here, "baseline_comparison.png")
        plt.savefig(comp_plot_path, dpi=150)
        plt.close()
        print(f"[Plots] Saved baseline metrics comparison chart -> {comp_plot_path}")
    else:
        print("[Plots] No global CSV database found. Skipping comparison chart.")

    # 2. Generate Trajectory Plot for latest run
    metrics_files = glob.glob(os.path.join(here, "*", "metrics.json"))
    if not metrics_files:
        print("[Plots] No metrics.json files found. Skipping trajectory overlay.")
        return
        
    metrics_files.sort(key=os.path.getmtime)
    latest_json_path = metrics_files[-1]
    latest_run_dir = os.path.dirname(latest_json_path)
    
    print(f"[Plots] Loading latest run metrics for trajectory: {latest_json_path}")
    try:
        with open(latest_json_path) as f:
            data = json.load(f)
            
        step_log = data.get("step_log", [])
        buildings = data.get("buildings", [])
        
        if not step_log:
            print("[Plots] Warning: step_log is empty in metrics.json. Skipping trajectory overlay.")
            return
            
        # Draw 2D flight trajectory plot
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # 1. Draw arena layout (200m x 200m)
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
        ax.set_aspect('equal')
        ax.grid(True, which='both', color='#e0e0e0', linestyle='-', linewidth=0.5)
        
        # 2. Draw buildings
        for b in buildings:
            # Draw building circular footprint
            circle = patches.Circle((b["x"], b["y"]), b["r"], color='#808080', alpha=0.6, label='Building' if b == buildings[0] else "")
            ax.add_patch(circle)
            
            # Draw outer safety inflation zone (dashed circle)
            # Inflation is 5.0m
            safety_circle = patches.Circle((b["x"], b["y"]), b["r"] + 5.0, color='#ff0000', fill=False, linestyle='--', alpha=0.25, linewidth=0.8, label='Safety Margin (+5m)' if b == buildings[0] else "")
            ax.add_patch(safety_circle)
            
            # Label building names
            ax.text(b["x"], b["y"], b["name"][:5], fontsize=8, ha='center', va='center', color='#ffffff', fontweight='bold')
            
        # 3. Plot Drone Path
        xs = [step["x"] for step in step_log]
        ys = [step["y"] for step in step_log]
        
        ax.plot(xs, ys, color='#1f77b4', linewidth=2, alpha=0.9, label='Flight Path')
        
        # Mark Start and Target Points
        start_pos = data.get("start_pos", [-40, 48, 15])
        target_pos = data.get("target_pos", [50, 48, 15])
        ax.scatter(start_pos[0], start_pos[1], color='#2ca02c', s=120, zorder=5, marker='o', label='Takeoff (Start)')
        ax.scatter(target_pos[0], target_pos[1], color='#d62728', s=200, zorder=5, marker='*', label='Destination (Target)')
        
        # Title & labels
        ax.set_title(f"Flight Trajectory & Collision Avoidance (Steps: {data.get('total_steps')})", fontsize=14, fontweight='bold')
        ax.set_xlabel("X coordinate (m)", fontsize=11)
        ax.set_ylabel("Y coordinate (m)", fontsize=11)
        
        # Legend
        ax.legend(loc='upper right', framealpha=0.9)
        
        # Save plots
        traj_plot_path = os.path.join(here, "flight_trajectory.png")
        plt.savefig(traj_plot_path, dpi=150, bbox_inches='tight')
        
        # Also archive it inside the specific run directory!
        run_traj_path = os.path.join(latest_run_dir, "flight_trajectory.png")
        plt.savefig(run_traj_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"[Plots] Saved flight trajectory overlay -> {traj_plot_path}")
        print(f"[Plots] Archived flight trajectory overlay -> {run_traj_path}")
        
    except Exception as e:
        print(f"[Plots] Error generating trajectory overlay: {e}")

if __name__ == "__main__":
    generate_plots()
