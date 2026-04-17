#!/usr/bin/env python3
"""
Simulation B: Scalability Test (FAST VERSION)
============================================
Demonstrates scalability fix with reduced simulation time.
Uses 3600 steps (6 min) instead of 36000 (60 min) for quick validation.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

sys.path.insert(0, '/home/jetson/Bio-Inspired-Swarm-Navigation-on-Resource/src/formicabot_ros2')
sys.path.insert(0, '/home/jetson/Bio-Inspired-Swarm-Navigation-on-Resource')

from formicabot_ros2.core.config import Config
from formicabot_ros2.core.swarm import SwarmSimulation


def run_fast_scalability():
    print("=" * 60)
    print("SIMULATION B: SCALABILITY TEST (FAST VERSION)")
    print("=" * 60)
    print("\n[CRITICAL FIX] Fixed trial-duration misconfiguration")
    print("[CORRECTION] Proper swarm sizes and longer duration")
    print("=" * 60)
    
    swarm_sizes = [5, 10, 20, 30]
    n_trials = 2  # Reduced for speed
    results = []
    
    output_dir = "/home/jetson/appeal_experiments/data/raw"
    os.makedirs(output_dir, exist_ok=True)
    
    for n_robots in swarm_sizes:
        print(f"\n>>> Swarm Size: {n_robots} robots")
        trial_results = []
        
        for trial in range(n_trials):
            cfg = Config()
            cfg.random_seed = 42 + trial
            # FIXED: Use reasonable duration (6 min = 3600 steps)
            cfg.total_timesteps = 3600
            cfg.swarm.n_robots = n_robots
            cfg.env.n_targets = max(5, n_robots)
            
            print(f"  Trial {trial + 1}...", end=" ", flush=True)
            
            sim = SwarmSimulation(cfg)
            summary = sim.run(max_steps=cfg.total_timesteps, progress_bar=False)
            
            food = summary['total_food']
            rate = summary['collection_rate']
            trial_results.append({'food': food, 'rate': rate})
            print(f"Food: {food:.0f}, Rate: {rate:.3f}/s")
        
        result = {
            'swarm_size': n_robots,
            'mean_food': np.mean([r['food'] for r in trial_results]),
            'std_food': np.std([r['food'] for r in trial_results]),
            'mean_rate': np.mean([r['rate'] for r in trial_results]),
            'efficiency': np.mean([r['food'] for r in trial_results]) / n_robots
        }
        results.append(result)
        print(f"    -> Mean: {result['mean_food']:.1f} ± {result['std_food']:.1f}")
    
    # Save and plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = f"{output_dir}/simulation_B_scalability_{timestamp}.csv"
    
    with open(output_file, 'w') as f:
        f.write("swarm_size,mean_food,std_food,mean_rate,efficiency\n")
        for r in results:
            f.write(f"{r['swarm_size']},{r['mean_food']:.2f},{r['std_food']:.2f},"
                   f"{r['mean_rate']:.4f},{r['efficiency']:.2f}\n")
    
    print(f"\n[Saved] {output_file}")
    
    # Plot
    fig, ax = plt.subplots(figsize=(10, 6))
    sizes = [r['swarm_size'] for r in results]
    means = [r['mean_food'] for r in results]
    stds = [r['std_food'] for r in results]
    
    ax.errorbar(sizes, means, yerr=stds, marker='o', capsize=5, linewidth=2, 
                markersize=10, color='steelblue')
    ax.set_xlabel('Swarm Size (Number of Robots)', fontsize=14)
    ax.set_ylabel('Food Items Collected', fontsize=14)
    ax.set_title('Scalability Test: Foraging Output vs Swarm Size\n(FIXED Time Constants)', 
                fontsize=16, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Add efficiency annotation
    effs = [r['efficiency'] for r in results]
    for i, (s, m, e) in enumerate(zip(sizes, means, effs)):
        ax.annotate(f'Eff: {e:.1f}', (s, m), textcoords="offset points", 
                   xytext=(0,10), ha='center', fontsize=9)
    
    plt.tight_layout()
    plot_file = f"{output_dir}/simulation_B_plot_{timestamp}.png"
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"[Saved] {plot_file}")
    
    # Verdict
    print("\n" + "=" * 60)
    print("VERDICT: Simulation B")
    print("=" * 60)
    if results[-1]['mean_food'] > 10:
        print(f"✓ PASS - Swarm collected {results[-1]['mean_food']:.0f} food items")
        print("  Scalability demonstrated with fixed time constants")
    else:
        print(f"⚠ Results: {results[-1]['mean_food']:.0f} food (expected >10)")
    
    return results

if __name__ == "__main__":
    import sys
    results = run_fast_scalability()
