"""
Batch Simulation Script for LNPCG Parameter Studies
Author: Rafael Coimbra Azeiteiro
Description: Runs parameter sweeps on LNPCG guidance to analyze:
             1. Effect of final bank angle (sigma_f) at fixed activation time
             2. Effect of activation time at fixed final bank angle
"""

import numpy as np
import matplotlib.pyplot as plt
from main import (
    MarsConstants, VehicleProperties, InitialConditions, 
    TargetConditions, PathConstraints, ReentrySimulation
)
from guidance import LNPCGuidance


def run_sigmaf_sweep():
    """
    Batch 1: Vary sigma_f from 10° to 90° with activation time fixed at 175s
    """
    print("=" * 80)
    print("BATCH 1: Varying Final Bank Angle (sigma_f)")
    print("=" * 80)
    
    # Fixed parameters
    activation_time = 175.0  # seconds
    sigmaf_values = [10, 30, 50, 70, 90]  # degrees
    
    # Initialize components (shared across all runs)
    mars = MarsConstants()
    vehicle = VehicleProperties()
    ic = InitialConditions()
    target = TargetConditions()
    constraints = PathConstraints()
    
    # Terminal energy
    e_f = mars.mu / (mars.radius + target.h) - 0.5 * target.V**2
    
    # Storage for results
    results_dict = {}
    colors = plt.cm.viridis(np.linspace(0, 1, len(sigmaf_values)))
    
    # Run simulations for each sigma_f
    for idx, sigmaf in enumerate(sigmaf_values):
        print(f"\nRunning simulation {idx+1}/{len(sigmaf_values)}: sigma_f = {sigmaf}°")
        
        # Create guidance
        guidance = LNPCGuidance(
            sigma_f_deg=float(sigmaf),
            e_f=e_f,
            activation_time=activation_time,
            max_iter=25,
            epsilon=100.0,
            de=10000.0,
            dsigma_deg=3.0
        )
        
        # Create simulation
        sim = ReentrySimulation(mars, vehicle, ic, target, constraints)
        
        # Run simulation
        results = sim.run(
            start_time=0.0, 
            stop_time=7000.0, 
            time_step=0.01, 
            guidance=guidance, 
            guidance_dt=1.0
        )
        
        # Store results
        results_dict[sigmaf] = {
            'results': results,
            'color': colors[idx]
        }
        
        print(f"  Completed in {results['t'][-1]:.2f}s")
        print(f"  Final altitude: {results['h'][-1]/1e3:.2f} km")
        print(f"  Final velocity: {results['V'][-1]:.2f} m/s")
    
    # Create comparison plots
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle(f'LNPCG Parameter Study: Varying σ_f (Activation Time = {activation_time}s)', 
                 fontsize=14, fontweight='bold')
    
    # Plot 1: Bank Angle vs Time
    ax1 = axes[0]
    for sigmaf, data in results_dict.items():
        res = data['results']
        ax1.plot(res['t'], np.rad2deg(res['sigma']), 
                label=f'σ_f = {sigmaf}°', 
                color=data['color'], 
                linewidth=2)
    ax1.set_xlabel('Time [s]', fontsize=11)
    ax1.set_ylabel('Bank Angle [deg]', fontsize=11)
    ax1.set_title('Bank Angle Command', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)
    
    # Plot 2: Altitude vs Velocity
    ax2 = axes[1]
    for sigmaf, data in results_dict.items():
        res = data['results']
        ax2.plot(res['V'], res['h']/1e3, 
                label=f'σ_f = {sigmaf}°', 
                color=data['color'], 
                linewidth=2)
    ax2.set_xlabel('Velocity [m/s]', fontsize=11)
    ax2.set_ylabel('Altitude [km]', fontsize=11)
    ax2.set_title('Altitude vs Velocity', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)
    ax2.invert_xaxis()  # Higher velocity on the left
    
    # Plot 3: Energy Dissipation Rate vs Time
    ax3 = axes[2]
    for sigmaf, data in results_dict.items():
        res = data['results']
        # Calculate energy dissipation rate: de/dt = -D*V [J/kg/s]
        # Already have energy, so compute derivative
        e_dot = np.gradient(res['energy'], res['t'])
        ax3.plot(res['t'], -e_dot/1e3,  # Convert to kJ/kg/s and make positive
                label=f'σ_f = {sigmaf}°', 
                color=data['color'], 
                linewidth=2)
    ax3.set_xlabel('Time [s]', fontsize=11)
    ax3.set_ylabel('Energy Dissipation Rate [kJ/kg/s]', fontsize=11)
    ax3.set_title('Energy Dissipation Rate', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best', fontsize=9)
    
    fig.tight_layout()
    
    return fig, results_dict


def run_activation_time_sweep():
    """
    Batch 2: Vary activation time from 115s to 175s with sigma_f fixed at 40°
    """
    print("\n" + "=" * 80)
    print("BATCH 2: Varying Activation Time")
    print("=" * 80)
    
    # Fixed parameters
    sigmaf = 40.0  # degrees
    activation_times = [115, 130, 145, 160, 175]  # seconds
    
    # Initialize components (shared across all runs)
    mars = MarsConstants()
    vehicle = VehicleProperties()
    ic = InitialConditions()
    target = TargetConditions()
    constraints = PathConstraints()
    
    # Terminal energy
    e_f = mars.mu / (mars.radius + target.h) - 0.5 * target.V**2
    
    # Storage for results
    results_dict = {}
    colors = plt.cm.plasma(np.linspace(0, 1, len(activation_times)))
    
    # Run simulations for each activation time
    for idx, t_act in enumerate(activation_times):
        print(f"\nRunning simulation {idx+1}/{len(activation_times)}: t_activation = {t_act}s")
        
        # Create guidance
        guidance = LNPCGuidance(
            sigma_f_deg=sigmaf,
            e_f=e_f,
            activation_time=float(t_act),
            max_iter=25,
            epsilon=100.0,
            de=10000.0,
            dsigma_deg=3.0
        )
        
        # Create simulation
        sim = ReentrySimulation(mars, vehicle, ic, target, constraints)
        
        # Run simulation
        results = sim.run(
            start_time=0.0, 
            stop_time=7000.0, 
            time_step=0.01, 
            guidance=guidance, 
            guidance_dt=1.0
        )
        
        # Store results
        results_dict[t_act] = {
            'results': results,
            'color': colors[idx]
        }
        
        print(f"  Completed in {results['t'][-1]:.2f}s")
        print(f"  Final altitude: {results['h'][-1]/1e3:.2f} km")
        print(f"  Final velocity: {results['V'][-1]:.2f} m/s")
    
    # Create comparison plots
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle(f'LNPCG Parameter Study: Varying Activation Time (σ_f = {sigmaf}°)', 
                 fontsize=14, fontweight='bold')
    
    # Plot 1: Bank Angle vs Time
    ax1 = axes[0]
    for t_act, data in results_dict.items():
        res = data['results']
        ax1.plot(res['t'], np.rad2deg(res['sigma']), 
                label=f't_act = {t_act}s', 
                color=data['color'], 
                linewidth=2)
    ax1.set_xlabel('Time [s]', fontsize=11)
    ax1.set_ylabel('Bank Angle [deg]', fontsize=11)
    ax1.set_title('Bank Angle Command', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)
    
    # Plot 2: Altitude vs Velocity
    ax2 = axes[1]
    for t_act, data in results_dict.items():
        res = data['results']
        ax2.plot(res['V'], res['h']/1e3, 
                label=f't_act = {t_act}s', 
                color=data['color'], 
                linewidth=2)
    ax2.set_xlabel('Velocity [m/s]', fontsize=11)
    ax2.set_ylabel('Altitude [km]', fontsize=11)
    ax2.set_title('Altitude vs Velocity', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)
    ax2.invert_xaxis()  # Higher velocity on the left
    
    # Plot 3: Energy Dissipation Rate vs Time
    ax3 = axes[2]
    for t_act, data in results_dict.items():
        res = data['results']
        # Calculate energy dissipation rate: de/dt = -D*V [J/kg/s]
        e_dot = np.gradient(res['energy'], res['t'])
        ax3.plot(res['t'], -e_dot/1e3,  # Convert to kJ/kg/s and make positive
                label=f't_act = {t_act}s', 
                color=data['color'], 
                linewidth=2)
    ax3.set_xlabel('Time [s]', fontsize=11)
    ax3.set_ylabel('Energy Dissipation Rate [kJ/kg/s]', fontsize=11)
    ax3.set_title('Energy Dissipation Rate', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best', fontsize=9)
    
    fig.tight_layout()
    
    return fig, results_dict


def main():
    """Run both batch simulations"""
    print("\n" + "=" * 80)
    print("LNPCG Batch Simulation Study")
    print("Testing parameter sensitivity for Mars reentry guidance")
    print("=" * 80)
    
    # Run Batch 1: Vary sigma_f
    fig1, results1 = run_sigmaf_sweep()
    
    # Run Batch 2: Vary activation time
    fig2, results2 = run_activation_time_sweep()
    
    # Display summary
    print("\n" + "=" * 80)
    print("SIMULATION SUMMARY")
    print("=" * 80)
    
    print("\nBatch 1 - Varying Final Bank Angle (σ_f):")
    print("  Activation Time: 175s")
    print("  σ_f values tested: 10°, 30°, 50°, 70°, 90°")
    print(f"  Total runs: {len(results1)}")
    
    print("\nBatch 2 - Varying Activation Time:")
    print("  Final Bank Angle: 40°")
    print("  Activation times tested: 115s, 130s, 145s, 160s, 175s")
    print(f"  Total runs: {len(results2)}")
    
    print("\n" + "=" * 80)
    print("Displaying results...")
    print("=" * 80)
    
    # Show plots
    plt.show()


if __name__ == "__main__":
    main()
