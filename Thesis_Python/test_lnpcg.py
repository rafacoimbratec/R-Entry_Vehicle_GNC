"""
Example: Running LNPCG Guidance

This demonstrates how to use the LNPCG guidance law
"""

import numpy as np
import matplotlib.pyplot as plt
from main import (MarsConstants, VehicleProperties, InitialConditions, 
                  TargetConditions, PathConstraints, ReentrySimulation)
from guidance import LNPCGuidance, ConstantBankGuidance


def main():
    print("=" * 70)
    print("LNPCG Guidance Demonstration")
    print("=" * 70)
    
    # Initialize components
    mars = MarsConstants()
    vehicle = VehicleProperties()
    ic = InitialConditions()
    target = TargetConditions()
    constraints = PathConstraints()
    
    # Create simulation
    sim = ReentrySimulation(mars, vehicle, ic, target, constraints)
    
    # Create LNPCG guidance
    guidance = LNPCGuidance(
        sigma_f_deg=10.0,        # Final bank angle at terminal
        e_f= mars.mu/(target.h+mars.radius)-0.5*target.V**2,             # Terminal energy [J/kg]
        activation_time=170.0,  # Activate after 170 seconds
        max_iter=50,            # Newton-Raphson iterations
        epsilon=100.0,          # Convergence tolerance [m]
        de=100.0,               # Energy step [J/kg]
        dsigma_deg=10.0         # Bank step for finite diff [deg]
    )
    
    print(f"\nGuidance: {guidance}")
    print(f"\nInitial Conditions:")
    print(f"  Altitude: {ic.h/1e3:.2f} km")
    print(f"  Velocity: {ic.V:.2f} m/s")
    print(f"  Latitude: {np.rad2deg(ic.phi):.2f} deg")
    print(f"  Longitude: {np.rad2deg(ic.theta):.2f} deg")
    
    print(f"\nTarget:")
    print(f"  Latitude: {np.rad2deg(target.phi):.2f} deg")
    print(f"  Longitude: {np.rad2deg(target.theta):.2f} deg")
    
    print("\n" + "=" * 70)
    print("Running simulation with LNPCG...")
    print("=" * 70)
    
    # Run simulation
    results = sim.run(
        start_time=0.0,
        stop_time=7000.0,
        time_step=0.1,
        guidance=guidance
    )
    
    print(f"\n{'=' * 70}")
    print("Simulation Complete!")
    print("=" * 70)
    print(f"  Flight time: {results['t'][-1]:.2f} s")
    print(f"  Final altitude: {results['h'][-1]/1e3:.2f} km")
    print(f"  Final velocity: {results['V'][-1]:.2f} m/s")
    print(f"  Final latitude: {np.rad2deg(results['phi'][-1]):.2f} deg")
    print(f"  Final longitude: {np.rad2deg(results['theta'][-1]):.2f} deg")
    
    # Check constraints
    print("\nConstraint Check:")
    q_max = np.max(results['q'])
    A_max = np.max(results['A'])
    Q_max = np.max(results['Q_dot'])
    
    print(f"  Max Dynamic Pressure: {q_max/1e3:.2f} kPa (limit: {constraints.q_max/1e3:.2f} kPa)")
    print(f"  Max Acceleration: {A_max/9.81:.2f} g (limit: {constraints.A_max/9.81:.2f} g)")
    print(f"  Max Heat Flux: {Q_max/1e3:.2f} kW/m² (limit: {constraints.Qdot_max/1e3:.2f} kW/m²)")
    
    # Plot results
    print("\nGenerating plots...")
    fig_states, fig_constraints = sim.plot_results(results)
    
    # Additional plot: Bank angle evolution
    fig_bank, ax = plt.subplots(figsize=(10, 6))
    ax.plot(results['t'], np.rad2deg(results['sigma']), 'b-', linewidth=2)
    ax.axvline(x=guidance.activation_time, color='r', linestyle='--', 
               label=f'Guidance Activation (t={guidance.activation_time}s)')
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Bank Angle [deg]', fontsize=12)
    ax.set_title('LNPCG Bank Angle Command', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    
    plt.show()


if __name__ == "__main__":
    main()
