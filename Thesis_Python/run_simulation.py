"""
Example script demonstrating modular guidance architecture

This shows how to:
1. Create a guidance object
2. Pass it to the simulation
3. Run multiple guidance strategies and compare
"""

import numpy as np
import matplotlib.pyplot as plt
from main import (MarsConstants, VehicleProperties, InitialConditions, 
                  TargetConditions, PathConstraints, ReentrySimulation)
from guidance import ConstantBankGuidance


def run_with_guidance(guidance_law, sim_params=None):
    """
    Run a simulation with a given guidance law
    
    Args:
        guidance_law: Instance of BaseGuidance subclass
        sim_params: Optional dict with 'start_time', 'stop_time', 'time_step'
    
    Returns:
        results: Simulation results dictionary
        sim: ReentrySimulation object
    """
    # Default simulation parameters
    if sim_params is None:
        sim_params = {
            'start_time': 0.0,
            'stop_time': 7000.0,
            'time_step': 0.1
        }
    
    # Initialize components (same as main.py)
    mars = MarsConstants()
    vehicle = VehicleProperties()
    ic = InitialConditions()
    target = TargetConditions()
    constraints = PathConstraints()
    
    # Create simulation
    sim = ReentrySimulation(mars, vehicle, ic, target, constraints)
    
    # Print header
    print("=" * 70)
    print(f"Running with: {guidance_law}")
    print("=" * 70)
    
    # Run simulation with guidance law
    results = sim.run(guidance=guidance_law, **sim_params)
    
    # Print summary
    print(f"\nSimulation completed in {results['t'][-1]:.2f} seconds")
    print(f"  Final altitude: {results['h'][-1]/1e3:.2f} km")
    print(f"  Final velocity: {results['V'][-1]:.2f} m/s")
    print(f"  Final latitude: {np.rad2deg(results['phi'][-1]):.2f} deg")
    print(f"  Final longitude: {np.rad2deg(results['theta'][-1]):.2f} deg")
    
    return results, sim


def compare_guidance_laws(guidance_list, sim_params=None):
    """
    Run multiple guidance laws and compare results
    
    Args:
        guidance_list: List of guidance law objects to compare
        sim_params: Optional simulation parameters
    
    Returns:
        results_dict: Dictionary mapping guidance names to results
    """
    results_dict = {}
    
    for guidance in guidance_list:
        # Reset guidance state
        guidance.reset()
        
        # Run simulation
        results, sim = run_with_guidance(guidance, sim_params)
        results_dict[guidance.name] = {
            'results': results,
            'guidance': guidance,
            'sim': sim
        }
    
    return results_dict


def plot_comparison(results_dict):
    """
    Plot comparison of multiple guidance strategies
    
    Args:
        results_dict: Dictionary from compare_guidance_laws()
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Guidance Strategy Comparison', fontsize=16)
    
    for name, data in results_dict.items():
        results = data['results']
        
        # Altitude vs time
        axes[0, 0].plot(results['t'], results['h']/1e3, label=name)
        
        # Velocity vs time
        axes[0, 1].plot(results['t'], results['V'], label=name)
        
        # Bank angle vs time
        axes[1, 0].plot(results['t'], np.rad2deg(results['sigma']), label=name)
        
        # Downrange (lat/lon trajectory)
        axes[1, 1].plot(np.rad2deg(results['theta']), 
                       np.rad2deg(results['phi']), label=name)
    
    # Format plots
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Altitude [km]')
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Velocity [m/s]')
    axes[0, 1].grid(True)
    axes[0, 1].legend()
    
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Bank Angle [deg]')
    axes[1, 0].grid(True)
    axes[1, 0].legend()
    
    axes[1, 1].set_xlabel('Longitude [deg]')
    axes[1, 1].set_ylabel('Latitude [deg]')
    axes[1, 1].grid(True)
    axes[1, 1].legend()
    axes[1, 1].set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    return fig


if __name__ == "__main__":
    # Example 1: Run a single guidance law
    print("\n" + "="*70)
    print("EXAMPLE 1: Single Guidance Law")
    print("="*70)
    
    guidance = ConstantBankGuidance(bank_angle_deg=30.0)
    results, sim = run_with_guidance(guidance)
    
    # Plot detailed results (from main.py)
    fig_states, fig_constraints = sim.plot_results(results)
    
    # Example 2: Compare multiple guidance laws
    print("\n" + "="*70)
    print("EXAMPLE 2: Comparing Multiple Guidance Laws")
    print("="*70)
    
    guidance_laws = [
        ConstantBankGuidance(bank_angle_deg=0.0),
        ConstantBankGuidance(bank_angle_deg=30.0),
        ConstantBankGuidance(bank_angle_deg=60.0),
    ]
    
    # Run all and compare
    results_dict = compare_guidance_laws(guidance_laws)
    
    # Plot comparison
    fig_comparison = plot_comparison(results_dict)
    
    plt.show()
    
    print("\n" + "="*70)
    print("All simulations complete!")
    print("="*70)
