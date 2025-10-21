"""
3DOF Reentry Vehicle Dynamics Model for Mars Entry
Author: Rafael Coimbra Azeiteiro
Description: Implements a 3-Degree-of-Freedom model for reentry vehicle guidance
             with fixed-step RK4 integration
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class MarsConstants:
    """Mars planetary constants"""
    radius: float = 3396.2e3  # Mars radius [m]
    mu: float = 4.282837e13  # Gravitational parameter [m^3/s^2]
    omega: float = 7.088e-5  # Planetary spin rate [rad/s]
    
    # Atmospheric parameters
    alpha1: float = 559.35
    alpha2: float = 188.95
    H_s: float = 11100  # Scale height [m]
    rho0: float = 0.020  # Surface density [kg/m^3]
    
    # Thermodynamic properties
    specific_heat_ratio: float = 1.29  # Cp/Cv for CO2
    R_star: float = 8314.32  # Universal gas constant [J/(kmol*K)]
    M_CO2: float = 44.01  # Molecular mass of CO2 [kg/kmol]
    
    @property
    def R_s(self) -> float:
        """Specific gas constant for Mars atmosphere [J/(kg*K)]"""
        return self.R_star / self.M_CO2


@dataclass
class VehicleProperties:
    """Entry vehicle aerodynamic properties"""
    L_over_D: float = 0.54  # Lift-to-drag ratio
    beta: float = 379  # Ballistic coefficient [kg/m^2]
    
    # Heat flux model constants
    N: float = 0.5
    M: float = 3.15
    k_heat_flux: float = 5.3697e-5


@dataclass
class InitialConditions:
    """Initial state of the reentry vehicle"""
    h: float = 125e3  # Initial altitude [m]
    theta: float = -176.40167 * np.pi/180  # Initial longitude [rad]
    phi: float = -21.3 * np.pi/180  # Initial latitude [rad]
    V: float = 4700  # Initial velocity [m/s]
    gamma: float = -10 * np.pi/180  # Initial flight path angle [rad]
    psi: float = -2.8758 * np.pi/180  # Initial heading angle [rad]


@dataclass
class TargetConditions:
    """Target conditions for the reentry vehicle"""
    theta: float = -175.8 * np.pi/180  # Target longitude [rad]
    phi: float = 0.276 * np.pi/180  # Target latitude [rad]
    V: float = 450  # Final velocity [m/s]
    h: float = 2480  # Final altitude [m]
    Rgo_target: float = 5e3  # Maximum allowable targeting error [m]


@dataclass
class PathConstraints:
    """Path constraints for the reentry vehicle"""
    A_max: float = 4  # Maximum acceleration [m/s^2] (4g in m/s^2)
    q_max: float = 13e3  # Maximum dynamic pressure [Pa]
    Qdot_max: float = 500e3  # Maximum heat rate [W/m^2]


class MarsAtmosphere:
    """Mars atmospheric model"""
    
    def __init__(self, mars: MarsConstants):
        self.mars = mars
    
    def density(self, h: float) -> float:
        """
        Calculate atmospheric density at given altitude using exponential model
        
        Args:
            h: Altitude above Mars surface [m]
            
        Returns:
            rho: Atmospheric density [kg/m^3]
        """
        # Exponential atmosphere model for Mars
        rho = self.mars.rho0 * np.exp(-h / self.mars.H_s)
        return rho
    
    def temperature(self, h: float) -> float:
        """
        Calculate atmospheric temperature (simplified model)
        
        Args:
            h: Altitude [m]
            
        Returns:
            T: Temperature [K]
        """
        # Old Lee et al. (2024) model
        h_km = h / 1e3
        T = 1.4e-13 * h_km**3 - 8.85e-9 * h_km**2 - 1.245e-3 * h_km + 205.36
        return T


class State:
    """Vehicle state vector [r, theta, phi, V, gamma, psi]"""
    
    def __init__(self, r: float, theta: float, phi: float, 
                 V: float, gamma: float, psi: float):
        self.r = r  # Radial distance from Mars center [m]
        self.theta = theta  # Longitude [rad]
        self.phi = phi  # Latitude [rad]
        self.V = V  # Velocity [m/s]
        self.gamma = gamma  # Flight path angle [rad]
        self.psi = psi  # Heading angle [rad]
    
    @property
    def h(self) -> float:
        """Altitude above surface [m]"""
        return self.r - 3396.2e3
    
    def to_array(self) -> np.ndarray:
        """Convert state to numpy array"""
        return np.array([self.r, self.theta, self.phi, self.V, self.gamma, self.psi])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'State':
        """Create state from numpy array"""
        return cls(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5])


class ReentryDynamics:
    """3DOF Reentry Vehicle Dynamics Model"""
    
    def __init__(self, mars: MarsConstants, vehicle: VehicleProperties, 
                 atmosphere: MarsAtmosphere):
        self.mars = mars
        self.vehicle = vehicle
        self.atmosphere = atmosphere
    
    def equations_of_motion(self, state: State, sigma: float) -> np.ndarray:
        """
        3DOF equations of motion for reentry vehicle
        
        Args:
            state: Current vehicle state
            sigma: Bank angle [rad]
            
        Returns:
            state_dot: Time derivative of state vector
        """
        r, theta, phi, V, gamma, psi = state.to_array()
        
        # Altitude above surface
        h = r - self.mars.radius
        
        # Atmospheric density
        rho = self.atmosphere.density(h)
        
        # Gravitational acceleration
        g = self.mars.mu / r**2
        
        # Dynamic pressure
        q = 0.5 * rho * V**2
        
        # Aerodynamic accelerations
        D = q / self.vehicle.beta  # Drag acceleration [m/s^2]
        L = self.vehicle.L_over_D * D  # Lift acceleration [m/s^2]
        
        # Centrifugal acceleration terms
        V_h = V * np.cos(gamma)  # Horizontal velocity
        
        # State derivatives
        r_dot = V * np.sin(gamma)
        
        theta_dot = (V_h * np.sin(psi)) / (r * np.cos(phi))
        
        phi_dot = (V_h * np.cos(psi)) / r
        
        V_dot = -D - g * np.sin(gamma) + self.mars.omega**2 * r * np.cos(phi) * \
                (np.sin(gamma) * np.cos(phi) - np.cos(gamma) * np.sin(phi) * np.sin(psi))
        
        gamma_dot = (L * np.cos(sigma) / V) + (V / r - g / V) * np.cos(gamma) + \
                    2 * self.mars.omega * np.cos(phi) * np.sin(psi) + \
                    (self.mars.omega**2 * r / V) * np.cos(phi) * \
                    (np.cos(gamma) * np.cos(phi) + np.sin(gamma) * np.sin(phi) * np.sin(psi))
        
        psi_dot = (L * np.sin(sigma)) / (V * np.cos(gamma)) + \
                  (V_h / r) * np.sin(psi) * np.tan(phi) - \
                  2 * self.mars.omega * (np.tan(gamma) * np.cos(phi) * np.cos(psi) - np.sin(phi)) + \
                  (self.mars.omega**2 * r / (V * np.cos(gamma))) * np.sin(phi) * np.cos(phi) * np.cos(psi)
        
        return np.array([r_dot, theta_dot, phi_dot, V_dot, gamma_dot, psi_dot])
    
    def rk4_step(self, state: State, sigma: float, dt: float) -> State:
        """
        Fixed-step 4th order Runge-Kutta integration
        
        Args:
            state: Current state
            sigma: Bank angle [rad]
            dt: Time step [s]
            
        Returns:
            next_state: State at next time step
        """
        y = state.to_array()
        
        # RK4 coefficients
        k1 = self.equations_of_motion(state, sigma)
        k2 = self.equations_of_motion(State.from_array(y + 0.5 * dt * k1), sigma)
        k3 = self.equations_of_motion(State.from_array(y + 0.5 * dt * k2), sigma)
        k4 = self.equations_of_motion(State.from_array(y + dt * k3), sigma)
        
        # Update state
        y_next = y + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        return State.from_array(y_next)
    
    def calculate_aerodynamic_quantities(self, state: State) -> dict:
        """Calculate aerodynamic quantities for constraint checking"""
        h = state.h
        rho = self.atmosphere.density(h)
        
        # Dynamic pressure
        q = 0.5 * rho * state.V**2
        
        # Drag and lift accelerations
        D = q / self.vehicle.beta
        L = self.vehicle.L_over_D * D
        
        # Total acceleration
        g = self.mars.mu / state.r**2
        A_total = np.sqrt(D**2 + L**2 + g**2)
        
        # Heat flux (Sutton-Graves correlation)
        Q_dot = self.vehicle.k_heat_flux * np.sqrt(rho) * state.V**self.vehicle.M
        
        return {
            'q': q,
            'A': A_total,
            'Q_dot': Q_dot,
            'D': D,
            'L': L,
            'rho': rho
        }


class ReentrySimulation:
    """Main simulation class for reentry vehicle"""
    
    def __init__(self, mars: MarsConstants, vehicle: VehicleProperties,
                 ic: InitialConditions, target: TargetConditions,
                 constraints: PathConstraints):
        self.mars = mars
        self.vehicle = vehicle
        self.ic = ic
        self.target = target
        self.constraints = constraints
        
        self.atmosphere = MarsAtmosphere(mars)
        self.dynamics = ReentryDynamics(mars, vehicle, self.atmosphere)
    
    def run(self, start_time: float = 0.0, stop_time: float = 7000.0,
            time_step: float = 0.01, bank_angle: float = 0.0) -> dict:
        """
        Run the reentry simulation
        
        Args:
            start_time: Start time [s]
            stop_time: Stop time [s]
            time_step: Time step [s]
            bank_angle: Constant bank angle for testing [rad]
            
        Returns:
            results: Dictionary containing simulation results
        """
        # Initialize state
        r0 = self.ic.h + self.mars.radius
        state = State(r0, self.ic.theta, self.ic.phi, 
                     self.ic.V, self.ic.gamma, self.ic.psi)
        
        # Time vector
        time = np.arange(start_time, stop_time, time_step)
        n_steps = len(time)
        
        # Preallocate arrays
        results = {
            't': time,
            'r': np.zeros(n_steps),
            'h': np.zeros(n_steps),
            'theta': np.zeros(n_steps),
            'phi': np.zeros(n_steps),
            'V': np.zeros(n_steps),
            'gamma': np.zeros(n_steps),
            'psi': np.zeros(n_steps),
            'q': np.zeros(n_steps),
            'A': np.zeros(n_steps),
            'Q_dot': np.zeros(n_steps),
            'rho': np.zeros(n_steps),
            'D': np.zeros(n_steps),
            'L': np.zeros(n_steps),
        }
        
        # Progress tracking
        progress_interval = max(1, n_steps // 10)  # Update every 10%
        
        # Simulation loop
        for i, t in enumerate(time):
            # Progress update
            if i % progress_interval == 0 and i > 0:
                print(f"  Progress: {100*i/n_steps:.1f}% (t={t:.1f}s, h={state.h/1e3:.1f}km)")
            
            # Store current state
            results['r'][i] = state.r
            results['h'][i] = state.h
            results['theta'][i] = state.theta
            results['phi'][i] = state.phi
            results['V'][i] = state.V
            results['gamma'][i] = state.gamma
            results['psi'][i] = state.psi
            
            # Calculate aerodynamic quantities
            aero = self.dynamics.calculate_aerodynamic_quantities(state)
            results['q'][i] = aero['q']
            results['A'][i] = aero['A']
            results['Q_dot'][i] = aero['Q_dot']
            results['rho'][i] = aero['rho']
            results['D'][i] = aero['D']
            results['L'][i] = aero['L']
            
            # Check termination conditions
            if state.h < self.target.h or state.h < 0:
                # Trim arrays to actual simulation length
                for key in results:
                    results[key] = results[key][:i+1]
                print(f"  Progress: 100.0% (simulation ended at t={t:.1f}s)")
                break
            
            # Integrate to next time step
            state = self.dynamics.rk4_step(state, bank_angle, time_step)
        
        return results
    
    def plot_results(self, results: dict):
        """Plot simulation results"""
        fig, axes = plt.subplots(3, 3, figsize=(15, 12))
        fig.suptitle('Mars Reentry Vehicle Simulation Results', fontsize=16)
        
        # Altitude
        axes[0, 0].plot(results['t'], results['h']/1e3)
        axes[0, 0].set_xlabel('Time [s]')
        axes[0, 0].set_ylabel('Altitude [km]')
        axes[0, 0].grid(True)
        axes[0, 0].axhline(y=self.target.h/1e3, color='r', linestyle='--', label='Target')
        axes[0, 0].legend()
        
        # Velocity
        axes[0, 1].plot(results['t'], results['V'])
        axes[0, 1].set_xlabel('Time [s]')
        axes[0, 1].set_ylabel('Velocity [m/s]')
        axes[0, 1].grid(True)
        axes[0, 1].axhline(y=self.target.V, color='r', linestyle='--', label='Target')
        axes[0, 1].legend()
        
        # Flight path angle
        axes[0, 2].plot(results['t'], np.rad2deg(results['gamma']))
        axes[0, 2].set_xlabel('Time [s]')
        axes[0, 2].set_ylabel('Flight Path Angle [deg]')
        axes[0, 2].grid(True)
        
        # Latitude
        axes[1, 0].plot(results['t'], np.rad2deg(results['phi']))
        axes[1, 0].set_xlabel('Time [s]')
        axes[1, 0].set_ylabel('Latitude [deg]')
        axes[1, 0].grid(True)
        axes[1, 0].axhline(y=np.rad2deg(self.target.phi), color='r', linestyle='--', label='Target')
        axes[1, 0].legend()
        
        # Longitude
        axes[1, 1].plot(results['t'], np.rad2deg(results['theta']))
        axes[1, 1].set_xlabel('Time [s]')
        axes[1, 1].set_ylabel('Longitude [deg]')
        axes[1, 1].grid(True)
        axes[1, 1].axhline(y=np.rad2deg(self.target.theta), color='r', linestyle='--', label='Target')
        axes[1, 1].legend()
        
        # Heading angle
        axes[1, 2].plot(results['t'], np.rad2deg(results['psi']))
        axes[1, 2].set_xlabel('Time [s]')
        axes[1, 2].set_ylabel('Heading Angle [deg]')
        axes[1, 2].grid(True)
        
        # Dynamic pressure
        axes[2, 0].plot(results['t'], results['q']/1e3)
        axes[2, 0].set_xlabel('Time [s]')
        axes[2, 0].set_ylabel('Dynamic Pressure [kPa]')
        axes[2, 0].grid(True)
        axes[2, 0].axhline(y=self.constraints.q_max/1e3, color='r', linestyle='--', label='Limit')
        axes[2, 0].legend()
        
        # Acceleration
        axes[2, 1].plot(results['t'], results['A']/9.81)
        axes[2, 1].set_xlabel('Time [s]')
        axes[2, 1].set_ylabel('Acceleration [g]')
        axes[2, 1].grid(True)
        axes[2, 1].axhline(y=self.constraints.A_max/9.81, color='r', linestyle='--', label='Limit')
        axes[2, 1].legend()
        
        # Heat flux
        axes[2, 2].plot(results['t'], results['Q_dot']/1e3)
        axes[2, 2].set_xlabel('Time [s]')
        axes[2, 2].set_ylabel('Heat Flux [kW/m²]')
        axes[2, 2].grid(True)
        axes[2, 2].axhline(y=self.constraints.Qdot_max/1e3, color='r', linestyle='--', label='Limit')
        axes[2, 2].legend()
        
        plt.tight_layout()
        return fig


def main():
    """Main function to run the simulation"""
    print("=" * 60)
    print("Mars Reentry Vehicle 3DOF Dynamics Simulation")
    print("=" * 60)
    
    # Initialize components
    mars = MarsConstants()
    vehicle = VehicleProperties()
    ic = InitialConditions()
    target = TargetConditions()
    constraints = PathConstraints()
    
    # Create simulation
    sim = ReentrySimulation(mars, vehicle, ic, target, constraints)
    
    print("\nInitial Conditions:")
    print(f"  Altitude: {ic.h/1e3:.2f} km")
    print(f"  Velocity: {ic.V:.2f} m/s")
    print(f"  Latitude: {np.rad2deg(ic.phi):.2f} deg")
    print(f"  Longitude: {np.rad2deg(ic.theta):.2f} deg")
    print(f"  Flight Path Angle: {np.rad2deg(ic.gamma):.2f} deg")
    print(f"  Heading Angle: {np.rad2deg(ic.psi):.2f} deg")
    
    print("\nTarget Conditions:")
    print(f"  Altitude: {target.h/1e3:.2f} km")
    print(f"  Velocity: {target.V:.2f} m/s")
    print(f"  Latitude: {np.rad2deg(target.phi):.2f} deg")
    print(f"  Longitude: {np.rad2deg(target.theta):.2f} deg")
    
    print("\nRunning simulation...")
    results = sim.run(start_time=0.0, stop_time=7000.0, time_step=0.1, bank_angle=0.0)
    
    print(f"\nSimulation completed in {results['t'][-1]:.2f} seconds")
    print(f"Final altitude: {results['h'][-1]/1e3:.2f} km")
    print(f"Final velocity: {results['V'][-1]:.2f} m/s")
    print(f"Final latitude: {np.rad2deg(results['phi'][-1]):.2f} deg")
    print(f"Final longitude: {np.rad2deg(results['theta'][-1]):.2f} deg")
    
    # Check constraints
    print("\nConstraint Violations:")
    q_max_sim = np.max(results['q'])
    A_max_sim = np.max(results['A'])
    Q_max_sim = np.max(results['Q_dot'])
    
    print(f"  Max Dynamic Pressure: {q_max_sim/1e3:.2f} kPa (limit: {constraints.q_max/1e3:.2f} kPa)")
    print(f"  Max Acceleration: {A_max_sim/9.81:.2f} g (limit: {constraints.A_max/9.81:.2f} g)")
    print(f"  Max Heat Flux: {Q_max_sim/1e3:.2f} kW/m² (limit: {constraints.Qdot_max/1e3:.2f} kW/m²)")
    
    # Plot results
    print("\nGenerating plots...")
    fig = sim.plot_results(results)
    plt.show()
    
    print("\nSimulation complete!")


if __name__ == "__main__":
    main()
