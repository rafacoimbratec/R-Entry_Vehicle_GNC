import random
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from spherical_metrics import spherical_metrics

# ============================================================
# Mars environment and vehicle constants
# ============================================================
class MarsEnv:
    radius = 3396.2e3          # [m]
    mu = 4.282837e13           # [m^3/s^2]
    omega = 7.088e-5           # [rad/s]
    Hs = 11100.0               # [m]
    rho0 = 0.020               # [kg/m^3]
    gamma_gas = 1.29
    R_star = 8314.32           # [J/(kmol*K)]
    M_CO2 = 44.01              # [kg/kmol]
    T_const = 210.0            # [K] (legacy default; not used in Mach now)

    def temperature(self, h: float) -> float:
        """
        Atmospheric temperature model (simplified; altitude-dependent).

        Args:
            h: Altitude above mean radius [m]

        Returns:
            Temperature [K]
        """
        h_km = h / 1e3
        T = 1.4e-13 * h_km**3 - 8.85e-9 * h_km**2 - 1.245e-3 * h_km + 205.36
        return T

    def sound_speed(self, h: float) -> float:
        """Speed of sound [m/s] at altitude h using temperature(h)."""
        Rspec = MarsEnv.R_star / MarsEnv.M_CO2
        T = self.temperature(h)
        T = max(1.0, float(T))  # guard against negative temps
        return np.sqrt(MarsEnv.gamma_gas * Rspec * T)


class Vehicle:
    L_over_D = 0.2483
    beta = 121.6            # ballistic coefficient as used in your eom
    N = 0.5                 # Sutton-Graves exponent on density
    M = 3.0                 # Sutton-Graves exponent on velocity
    k_heat_flux = 2.336276498787735e-4 # Sutton-Graves coefficient


@dataclass
class PathConstraints:
    """Path constraints for the reentry vehicle (g-load, dynamic pressure, heat rate)."""
    A_max: float = 12.0        # [g] maximum total acceleration
    q_max: float = 13e3       # [Pa] maximum dynamic pressure
    Qdot_max: float = 700e3   # [W/m^2] maximum heat rate


mars = MarsEnv()
veh = Vehicle()
constraints = PathConstraints()

# ============================================================
# Equations of motion
# State: [r, th(lon), ph(lat), V, gam, psi]
# ============================================================
def eom(state, sigma):
    r, th, ph, V, gam, psi = state
    g = mars.mu / r**2
    h = r - mars.radius
    rho = mars.rho0 * np.exp(-h / mars.Hs)

    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D
    Vh = V * np.cos(gam)

    r_dot = V * np.sin(gam)
    th_dot = (Vh * np.sin(psi)) / (r * np.cos(ph))
    ph_dot = (Vh * np.cos(psi)) / r

    V_dot = -D - g * np.sin(gam) + mars.omega**2 * r * np.cos(ph) * \
        (np.sin(gam)*np.cos(ph) - np.cos(gam)*np.sin(ph)*np.cos(psi))

    V_safe = max(V, 1e-6)
    cos_gam_safe = max(np.cos(gam), 1e-6)

    gam_dot = (L*np.cos(sigma)/V_safe) + (V/r - g/V_safe)*np.cos(gam) + \
              2*mars.omega*np.cos(ph)*np.sin(psi) + \
              (mars.omega**2*r/V_safe)*np.cos(ph)* \
              (np.cos(gam)*np.cos(ph) + np.sin(gam)*np.sin(ph)*np.cos(psi))

    psi_dot = (L*np.sin(sigma))/(V_safe*cos_gam_safe) + \
              (Vh/r)*np.sin(psi)*np.tan(ph) - \
              2*mars.omega*(np.tan(gam)*np.cos(ph)*np.cos(psi) - np.sin(ph)) + \
              (mars.omega**2*r/(V_safe*cos_gam_safe))*np.sin(ph)*np.cos(ph)*np.sin(psi)

    return np.array([r_dot, th_dot, ph_dot, V_dot, gam_dot, psi_dot])

# ============================================================
# RK4 integrator
# ============================================================
def rk4_step(f, state, sigma, dt):
    k1 = f(state, sigma)
    k2 = f(state + 0.5*dt*k1, sigma)
    k3 = f(state + 0.5*dt*k2, sigma)
    k4 = f(state + dt*k3, sigma)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

# ============================================================
# Energy calculation
# ============================================================
def compute_energy(r, V):
    """
    Compute specific mechanical energy [J/kg]
    E = -mu/r - 0.5*V^2
    """
    return -mars.mu / r - 0.5 * V**2

# ============================================================
# Logistic energy-based bank angle parameterization
# ============================================================
def sigma_logistic_ref(E, E0, Ef, sigma0, K, sigma_max):
    """
    Logistic energy-based bank angle parameterisation with saturation.
    
    Args:
        E: current specific mechanical energy [J/kg]
        E0: energy at entry interface [J/kg]
        Ef: reference final energy (near deploy) [J/kg]
        sigma0: initial bank angle level at E0 [rad]
        K: decay rate (larger K -> faster reduction of |sigma|)
        sigma_max: max allowed |sigma| for saturation [rad]
    
    Returns:
        sigma_ref: commanded bank angle [rad]
    """
    # Protect against division by zero
    if abs(Ef - E0) < 1e-6:
        sigma_cmd = sigma0
    else:
        # Normalised energy
        z = (E - E0) / (Ef - E0)
        # Logistic law (Lee-style)
        sigma_cmd = 2 * sigma0 / (1 + np.exp(K * z))
    
    # Symmetric saturation
    sigma_ref = np.clip(sigma_cmd, -sigma_max, sigma_max)
    
    return sigma_ref

# ============================================================
# Loads and deploy trigger (Mach/q rule)
# ============================================================
def compute_loads(state):
    """
    Given the current state [r, th, ph, V, gam, psi], compute:
    - altitude h
    - density rho
    - dynamic pressure q
    - total acceleration A_total (in g's)
    - heat rate Qdot
    """
    r, th, ph, V, gam, psi = state
    h = r - mars.radius
    rho = mars.rho0 * np.exp(-h / mars.Hs)

    # Aerodynamic accelerations (as in eom)
    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D

    # Dynamic pressure
    q = 0.5 * rho * V**2

    # Total acceleration in g's (D and L have units of acceleration)
    g0 = 9.80665
    A_total = np.sqrt(D**2 + L**2) / g0

    # Heat flux (Sutton–Graves correlation)
    Qdot = veh.k_heat_flux * (rho**veh.N) * (V**veh.M)

    return h, rho, q, A_total, Qdot


def deploy_trigger(h, V, q):
    """
    Deploy rule based on Mach, dynamic pressure, and altitude:

      - 1.4 <= Mach <= 2.2
      - 300 Pa <= q <= 800 Pa
      - h >= 6 km (6000 m)
      OR impact (h <= 0)
    """
    a = mars.sound_speed(h)
    a_safe = max(a, 1e-6)
    mach = V / a_safe

    return ((1.5 <= mach <= 2.5) and (300.0 <= q <= 800.0) or (h <= 6000.0))

# ============================================================
# Safe simulation with logistic bank profile
# ============================================================
def simulate_entry_safe(initial_state,
                        dt=0.25,
                        tmax=2000.0,
                        sigma0=np.deg2rad(75.0),
                        K=1.0,
                        sigma_max=np.deg2rad(120.0),
                        constraints=constraints):
    """
    Integrate one trajectory using logistic energy-based bank profile.

    Returns:
        state_f      : final state at stop time
        t_f          : final time
        violated     : True if any path constraint was violated
        max_q        : maximum dynamic pressure along the path [Pa]
        max_A        : maximum total acceleration along the path [g]
        max_Qdot     : maximum heat rate along the path [W/m^2]
    """
    # Compute initial and final energies
    r0, th0, ph0, V0, gam0, psi0 = initial_state
    E0 = compute_energy(r0, V0)
    
    # Final energy: altitude = 6 km, velocity = 475 m/s
    h_final = 6000.0  # [m]
    V_final = 475.0   # [m/s]
    r_final = mars.radius + h_final
    Ef = compute_energy(r_final, V_final)
    
    state = initial_state.copy()
    t = 0.0

    max_q = 0.0
    max_A = 0.0
    max_Qdot = 0.0
    violated = False

    while t < tmax:
        # Compute current energy
        r, th, ph, V, gam, psi = state
        E = compute_energy(r, V)
        
        # Compute bank angle using logistic law
        sigma = sigma_logistic_ref(E, E0, Ef, sigma0, K, sigma_max)
        
        # Integrate
        state = rk4_step(eom, state, sigma, dt)

        # Compute loads
        h, rho, q, A_total, Qdot = compute_loads(state)
        r, th, ph, V, gam, psi = state

        # Update maxima
        max_q = max(max_q, q)
        max_A = max(max_A, A_total)
        max_Qdot = max(max_Qdot, Qdot)

        # Check path constraints (but don't break)
        if (q > constraints.q_max or
            A_total > constraints.A_max or
            Qdot > constraints.Qdot_max):
            violated = True

        # Deploy trigger: Mach/q rule
        if deploy_trigger(h, V, q):
            break

        t += dt

    return state, t, violated, max_q, max_A, max_Qdot

# ============================================================
# Build reachable set by sweeping control parameters
# ============================================================
def build_reachable_set(lat0, lon0,
                        sigma0_grid, K_grid,
                        dt=0.25,
                        tmax=4000.0,
                        sigma_max=np.deg2rad(120.0)):
    """
    Build a reachable set by sweeping control parameters (sigma0, K).

    Args:
        lat0, lon0: entry point latitude and longitude [rad]
        sigma0_grid: array of sigma0 values to sweep [rad]
        K_grid: array of K values to sweep (decay rate)
        dt: integration timestep [s]
        tmax: maximum simulation time [s]
        sigma_max: maximum bank angle saturation [rad]

    Returns:
        all_samples : array of shape (N, 9) with columns
                      [lat_f_deg, lon_f_deg, ALT_km, max_q, max_A, max_Qdot, sigma0_deg, K, gam_f_deg]
    """
    all_samples = []

    # Fixed entry conditions
    h0 = 128e3
    r0 = mars.radius + h0
    V0 = 5500.0
    gam0 = np.deg2rad(-15.5)
    psi0 = np.deg2rad(90.0)

    state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])

    for sigma0 in sigma0_grid:
        for K in K_grid:
            state_f, tf, violated, max_q, max_A, max_Qdot = simulate_entry_safe(
                state0, dt=dt, tmax=tmax, sigma0=sigma0, K=K, sigma_max=sigma_max
            )

            r, th, ph, V, gam, psi = state_f
            h = r - mars.radius

            lat_f_deg = np.degrees(ph)
            lon_f_deg = np.degrees(th)
            ALT_km = h / 1000.0
            gam_f_deg = np.degrees(gam)
            sigma0_deg = np.degrees(sigma0)

            record = [lat_f_deg, lon_f_deg, ALT_km, max_q, max_A, max_Qdot, sigma0_deg, K, gam_f_deg]
            all_samples.append(record)
            
            status = "VIOLATED" if violated else "OK"
            print(
                f"[{status:8s}] σ0={sigma0_deg:6.2f}°, K={K:5.2f} | "
                f"lat={lat_f_deg:7.2f}°, lon={lon_f_deg:7.2f}°, ALT={ALT_km:6.2f} km, γ_f={gam_f_deg:6.2f}° | "
                f"max_q={max_q:8.1f} Pa, max_A={max_A:5.2f} g, max_Qdot={max_Qdot:8.1f} W/m^2"
            )

    all_samples = np.array(all_samples) if len(all_samples) > 0 else np.empty((0, 9))

    return all_samples

# ============================================================
# MAIN SCRIPT: Control parameter sweep and reachable set
# ============================================================
if __name__ == "__main__":
    deg2rad = np.pi/180
    lat0 = 20.83 * deg2rad
    lon0 = 66.8 * deg2rad

    # Sweep control parameters: sigma0 in range [-120°, +120°] and K in [0.1, 2.0]
    # Coarse mesh for speed (100 x 100 = 10000 simulations)
    sigma0_grid = np.linspace(np.deg2rad(-120), np.deg2rad(120), 20)
    K_grid = np.linspace(0.1, 3.0, 20)

    print(f"Building reachable set with {len(sigma0_grid)} x {len(K_grid)} = {len(sigma0_grid)*len(K_grid)} control parameter combinations")
    print(f"sigma0 range: [{np.degrees(sigma0_grid[0]):.1f}°, {np.degrees(sigma0_grid[-1]):.1f}°]")
    print(f"K range: [{K_grid[0]:.2f}, {K_grid[-1]:.2f}]")
    print("="*80)

    # ------------------------------------------------------------
    # Build reachable set
    # ------------------------------------------------------------
    all_samples = build_reachable_set(
        lat0, lon0, sigma0_grid, K_grid,
        dt=0.25, tmax=4000.0
    )

    # ------------------------------------------------------------
    # Plot reachable set: Lat/Lon footprint + path constraints
    # ------------------------------------------------------------
    if all_samples.shape[0] > 0:
        lat_all  = all_samples[:, 0]  # latitude [deg]
        lon_all  = all_samples[:, 1]  # longitude [deg]
        ALT_all  = all_samples[:, 2]  # altitude [km]
        q_all    = all_samples[:, 3]  # max dynamic pressure [Pa]
        A_all    = all_samples[:, 4]  # max acceleration [g]
        Qdot_all = all_samples[:, 5]  # max heat rate [W/m^2]
        gam_f_all = all_samples[:, 8]  # final flight path angle [deg]

        # Compute downrange and crossrange for each sample
        # Use a reference target slightly ahead of entry point for proper coordinate frame
        lat_ref = lat0 + np.deg2rad(1.0)  # 1 degree ahead in latitude
        lon_ref = lon0  # Same longitude
        
        RD_all = []
        RC_all = []
        for i in range(len(lat_all)):
            lat_f = np.deg2rad(lat_all[i])
            lon_f = np.deg2rad(lon_all[i])
            # Compute metrics relative to reference direction
            # Note: spherical_metrics outputs are swapped (RC is actually downrange, RD is crossrange)
            d_m, sin_d, cos_d, RD, RC, RD_go, Rgo = spherical_metrics(
                lat0, lon0, lat_f, lon_f, lat_ref, lon_ref, mars.radius
            )
            RD_all.append(RD / 1e3)  # Convert to km
            RC_all.append(RC / 1e3)  # Convert to km
        
        RD_all = np.array(RD_all)
        RC_all = np.array(RC_all)

        # ===== FIGURE 1: LAT/LON FOOTPRINT =====
        fig1, axes1 = plt.subplots(2, 2, figsize=(14, 10))

        # 1) Altitude contour on lat/lon
        levels_alt = 6
        cf0 = axes1[0,0].tricontourf(lon_all, lat_all, ALT_all, levels=levels_alt, cmap='viridis')
        cs0 = axes1[0,0].tricontour(lon_all, lat_all, ALT_all, levels=levels_alt, colors='k', linewidths=0.6)
        axes1[0,0].clabel(cs0, fmt="%.1f", fontsize=8)
        cbar0 = fig1.colorbar(cf0, ax=axes1[0,0], label='Altitude [km]')
        axes1[0,0].set_xlabel('Longitude [deg]')
        axes1[0,0].set_ylabel('Latitude [deg]')
        axes1[0,0].set_title('Deployment Altitude Footprint')
        axes1[0,0].grid(True, alpha=0.3)

        # 2) Maximum dynamic pressure on lat/lon
        levels_q = 6
        q_kPa = q_all / 1e3  # convert Pa to kPa
        cf1 = axes1[0,1].tricontourf(lon_all, lat_all, q_kPa, levels=levels_q, cmap='plasma')
        cs1 = axes1[0,1].tricontour(lon_all, lat_all, q_kPa, levels=levels_q, colors='k', linewidths=0.6)
        axes1[0,1].clabel(cs1, fmt="%.1f", fontsize=8)
        cbar1 = fig1.colorbar(cf1, ax=axes1[0,1], label='Max q [kPa]')
        axes1[0,1].text(0.02, 0.98, f'Limit: {constraints.q_max/1e3:.1f} kPa', 
                      transform=axes1[0,1].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes1[0,1].set_xlabel('Longitude [deg]')
        axes1[0,1].set_ylabel('Latitude [deg]')
        axes1[0,1].set_title('Maximum Dynamic Pressure')
        axes1[0,1].grid(True, alpha=0.3)

        # 3) Maximum acceleration on lat/lon
        levels_a = 6
        cf2 = axes1[1,0].tricontourf(lon_all, lat_all, A_all, levels=levels_a, cmap='hot')
        cs2 = axes1[1,0].tricontour(lon_all, lat_all, A_all, levels=levels_a, colors='k', linewidths=0.6)
        axes1[1,0].clabel(cs2, fmt="%.2f", fontsize=8)
        cbar2 = fig1.colorbar(cf2, ax=axes1[1,0], label='Max A [g]')
        axes1[1,0].text(0.02, 0.98, f'Limit: {constraints.A_max:.1f} g', 
                      transform=axes1[1,0].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes1[1,0].set_xlabel('Longitude [deg]')
        axes1[1,0].set_ylabel('Latitude [deg]')
        axes1[1,0].set_title('Maximum Acceleration')
        axes1[1,0].grid(True, alpha=0.3)

        # 4) Maximum heat rate on lat/lon
        levels_qd = 6
        Qdot_kW = Qdot_all / 1e3  # convert W/m^2 to kW/m^2
        cf3 = axes1[1,1].tricontourf(lon_all, lat_all, Qdot_kW, levels=levels_qd, cmap='inferno')
        cs3 = axes1[1,1].tricontour(lon_all, lat_all, Qdot_kW, levels=levels_qd, colors='k', linewidths=0.6)
        axes1[1,1].clabel(cs3, fmt="%.1f", fontsize=8)
        cbar3 = fig1.colorbar(cf3, ax=axes1[1,1], label='Max Q̇ [kW/m²]')
        axes1[1,1].text(0.02, 0.98, f'Limit: {constraints.Qdot_max/1e3:.0f} kW/m²', 
                      transform=axes1[1,1].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes1[1,1].set_xlabel('Longitude [deg]')
        axes1[1,1].set_ylabel('Latitude [deg]')
        axes1[1,1].set_title('Maximum Heat Rate')
        axes1[1,1].grid(True, alpha=0.3)

        plt.suptitle('Reachable Set (Logistic): Lat/Lon Footprint', fontsize=14, y=0.995)
        plt.tight_layout()

        # ===== FIGURE 2: DOWNRANGE/CROSSRANGE FOOTPRINT =====
        fig2, axes2 = plt.subplots(2, 2, figsize=(14, 10))

        # 1) Altitude contour on downrange/crossrange
        cf0_dr = axes2[0,0].tricontourf(RD_all, RC_all, ALT_all, levels=levels_alt, cmap='viridis')
        cs0_dr = axes2[0,0].tricontour(RD_all, RC_all, ALT_all, levels=levels_alt, colors='k', linewidths=0.6)
        axes2[0,0].clabel(cs0_dr, fmt="%.1f", fontsize=8)
        cbar0_dr = fig2.colorbar(cf0_dr, ax=axes2[0,0], label='Altitude [km]')
        axes2[0,0].set_xlabel('Downrange [km]')
        axes2[0,0].set_ylabel('Crossrange [km]')
        axes2[0,0].set_title('Deployment Altitude Footprint')
        axes2[0,0].grid(True, alpha=0.3)

        # 2) Maximum dynamic pressure on downrange/crossrange
        cf1_dr = axes2[0,1].tricontourf(RD_all, RC_all, q_kPa, levels=levels_q, cmap='plasma')
        cs1_dr = axes2[0,1].tricontour(RD_all, RC_all, q_kPa, levels=levels_q, colors='k', linewidths=0.6)
        axes2[0,1].clabel(cs1_dr, fmt="%.1f", fontsize=8)
        cbar1_dr = fig2.colorbar(cf1_dr, ax=axes2[0,1], label='Max q [kPa]')
        axes2[0,1].text(0.02, 0.98, f'Limit: {constraints.q_max/1e3:.1f} kPa', 
                      transform=axes2[0,1].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes2[0,1].set_xlabel('Downrange [km]')
        axes2[0,1].set_ylabel('Crossrange [km]')
        axes2[0,1].set_title('Maximum Dynamic Pressure')
        axes2[0,1].grid(True, alpha=0.3)

        # 3) Maximum acceleration on downrange/crossrange
        cf2_dr = axes2[1,0].tricontourf(RD_all, RC_all, A_all, levels=levels_a, cmap='hot')
        cs2_dr = axes2[1,0].tricontour(RD_all, RC_all, A_all, levels=levels_a, colors='k', linewidths=0.6)
        axes2[1,0].clabel(cs2_dr, fmt="%.2f", fontsize=8)
        cbar2_dr = fig2.colorbar(cf2_dr, ax=axes2[1,0], label='Max A [g]')
        axes2[1,0].text(0.02, 0.98, f'Limit: {constraints.A_max:.1f} g', 
                      transform=axes2[1,0].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes2[1,0].set_xlabel('Downrange [km]')
        axes2[1,0].set_ylabel('Crossrange [km]')
        axes2[1,0].set_title('Maximum Acceleration')
        axes2[1,0].grid(True, alpha=0.3)

        # 4) Maximum heat rate on downrange/crossrange
        cf3_dr = axes2[1,1].tricontourf(RD_all, RC_all, Qdot_kW, levels=levels_qd, cmap='inferno')
        cs3_dr = axes2[1,1].tricontour(RD_all, RC_all, Qdot_kW, levels=levels_qd, colors='k', linewidths=0.6)
        axes2[1,1].clabel(cs3_dr, fmt="%.1f", fontsize=8)
        cbar3_dr = fig2.colorbar(cf3_dr, ax=axes2[1,1], label='Max Q̇ [kW/m²]')
        axes2[1,1].text(0.02, 0.98, f'Limit: {constraints.Qdot_max/1e3:.0f} kW/m²', 
                      transform=axes2[1,1].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes2[1,1].set_xlabel('Downrange [km]')
        axes2[1,1].set_ylabel('Crossrange [km]')
        axes2[1,1].set_title('Maximum Heat Rate')
        axes2[1,1].grid(True, alpha=0.3)

        plt.suptitle('Reachable Set (Logistic): Downrange/Crossrange Footprint', fontsize=14, y=0.995)
        plt.tight_layout()

        # ============================================================
        # TARGET SELECTION: Apply cost function to choose best target
        # ============================================================
        print("\n" + "="*80)
        print("TARGET SELECTION")
        print("="*80)
        
            # Apply constraints to filter valid targets
        valid_mask = (
            (np.abs(RC_all) < 20.0) &          # Crossrange < 10 km
            (A_all < 12.0) &                    # Acceleration < 15 g
            (q_all <= constraints.q_max) &     # Dynamic pressure within limit
            (Qdot_all <= constraints.Qdot_max) # Heat rate within limit
        )
        
        n_valid = np.sum(valid_mask)
        print(f"Valid targets (meeting constraints): {n_valid} / {len(lat_all)}")
        
        if n_valid > 0:
            # Extract valid samples
            lat_valid = lat_all[valid_mask]
            lon_valid = lon_all[valid_mask]
            ALT_valid = ALT_all[valid_mask]
            RC_valid = RC_all[valid_mask]
            RD_valid = RD_all[valid_mask]
            q_valid = q_all[valid_mask]
            A_valid = A_all[valid_mask]
            Qdot_valid = Qdot_all[valid_mask]
            gam_f_valid = gam_f_all[valid_mask]
            sigma0_valid = all_samples[valid_mask, 6]
            K_valid = all_samples[valid_mask, 7]
            
            # Multi-objective cost function with normalized weights
            # Normalize each objective to [0, 1] range
            ALT_norm = (ALT_valid - ALT_valid.min()) / (ALT_valid.max() - ALT_valid.min() + 1e-9)
            A_norm = (A_valid - A_valid.min()) / (A_valid.max() - A_valid.min() + 1e-9)
            Qdot_norm = (Qdot_valid - Qdot_valid.min()) / (Qdot_valid.max() - Qdot_valid.min() + 1e-9)
            RC_norm = (np.abs(RC_valid) - np.abs(RC_valid).min()) / (np.abs(RC_valid).max() - np.abs(RC_valid).min() + 1e-9)
            gam_norm = (np.abs(gam_f_valid) - np.abs(gam_f_valid).min()) / (np.abs(gam_f_valid).max() - np.abs(gam_f_valid).min() + 1e-9)
            
            # Weighted cost (minimize):
            # - Maximize altitude (minimize -ALT)
            # - Minimize g-load
            # - Minimize heat load
            # - Minimize crossrange
            # - Minimize flight path angle magnitude
            w_alt = 0.05      # Weight for altitude (higher is better)
            w_A = 0.02       # Weight for acceleration (lower is better)
            w_Qdot = 0.03     # Weight for heat rate (lower is better)
            w_RC = 0.0       # Weight for crossrange (lower is better)
            w_gam = 0.9      # Weight for flight path angle (lower is better)
            
            cost = -w_alt * ALT_norm + w_A * A_norm + w_Qdot * Qdot_norm + w_RC * RC_norm + w_gam * gam_norm
            
            print(f"\nCost function weights:")
            print(f"  Altitude:           {w_alt:.2f} (maximize)")
            print(f"  Acceleration:       {w_A:.2f} (minimize)")
            print(f"  Heat Rate:          {w_Qdot:.2f} (minimize)")
            print(f"  Crossrange:         {w_RC:.2f} (minimize)")
            print(f"  Flight Path Angle:  {w_gam:.2f} (minimize)")
            
            # Find best target
            best_idx = np.argmin(cost)
            
            lat_target = lat_valid[best_idx]
            lon_target = lon_valid[best_idx]
            ALT_target = ALT_valid[best_idx]
            RC_target = RC_valid[best_idx]
            RD_target = RD_valid[best_idx]
            q_target = q_valid[best_idx]
            A_target = A_valid[best_idx]
            Qdot_target = Qdot_valid[best_idx]
            gam_f_target = gam_f_valid[best_idx]
            sigma0_best = sigma0_valid[best_idx]
            K_best = K_valid[best_idx]
            cost_target = cost[best_idx]
            
            print(f"\nBest target selected (cost = {cost_target:.4f}):")
            print(f"  Latitude:          {lat_target:7.3f}°")
            print(f"  Longitude:         {lon_target:7.3f}°")
            print(f"  Downrange:         {RD_target:7.2f} km")
            print(f"  Crossrange:        {RC_target:7.2f} km")
            print(f"  Altitude:          {ALT_target:7.2f} km")
            print(f"  Flight Path Angle: {gam_f_target:7.2f}°")
            print(f"  Max q:             {q_target:8.1f} Pa")
            print(f"  Max A:             {A_target:5.2f} g")
            print(f"  Max Qdot:          {Qdot_target:8.1f} W/m²")
            print(f"  Control: σ0={sigma0_best:6.2f}°, K={K_best:.3f}")
            
            # ============================================================
            # SIMULATE BEST TRAJECTORY WITH FULL TRACE
            # ============================================================
            print("\n" + "="*80)
            print("SIMULATING BEST TRAJECTORY")
            print("="*80)
            
            h0 = 128e3
            r0 = mars.radius + h0
            V0 = 5500.0
            gam0 = np.deg2rad(-15.5)
            psi0 = np.deg2rad(90.0)
            state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])
            
            # Compute initial and final energies
            E0 = compute_energy(r0, V0)
            h_final = 6000.0
            V_final = 475.0
            r_final = mars.radius + h_final
            Ef = compute_energy(r_final, V_final)
            
            sigma0_rad = np.deg2rad(sigma0_best)
            sigma_max_rad = np.deg2rad(120.0)
            
            dt_sim = 0.25
            tmax_sim = 4000.0
            
            state = state0.copy()
            t = 0.0
            times = [t]
            states = [state.copy()]
            sigmas = []
            q_hist = []
            A_hist = []
            Qdot_hist = []
            mach_hist = []
            
            while t < tmax_sim:
                # Compute current energy and bank angle
                r, th, ph, V, gam, psi = state
                E = compute_energy(r, V)
                sigma = sigma_logistic_ref(E, E0, Ef, sigma0_rad, K_best, sigma_max_rad)
                sigmas.append(sigma)
                
                # Integrate
                state = rk4_step(eom, state, sigma, dt_sim)
                t += dt_sim
                
                times.append(t)
                states.append(state.copy())
                
                # Compute loads
                h, rho, q, A_total, Qdot = compute_loads(state)
                q_hist.append(q)
                A_hist.append(A_total)
                Qdot_hist.append(Qdot)
                
                # Compute Mach number
                a = mars.sound_speed(h)
                mach = V / max(a, 1e-6)
                mach_hist.append(mach)
                
                r, th, ph, V, gam, psi = state
                if deploy_trigger(h, V, q):
                    print(f"Mach at deployment: {mach:.2f}")
                    print(f"Deployment triggered at t={t:.2f} s")
                    break
            
            times = np.array(times)
            states = np.array(states)
            sigmas = np.array(sigmas)
            q_hist = np.array(q_hist)
            A_hist = np.array(A_hist)
            Qdot_hist = np.array(Qdot_hist)
            mach_hist = np.array(mach_hist)
            
            # ============================================================
            # PLOT BEST TRAJECTORY
            # ============================================================
            
            # Extract trajectory data
            h_traj = (states[:, 0] - mars.radius) / 1e3  # Altitude [km]
            lon_traj = np.degrees(states[:, 1])
            lat_traj = np.degrees(states[:, 2])
            V_traj = states[:, 3]
            gam_traj = np.degrees(states[:, 4])
            psi_traj = np.degrees(states[:, 5])
            sigma_traj = np.degrees(sigmas)
            
            # Figure: Ground track
            fig_track, ax_track = plt.subplots(figsize=(10, 8))
            sc = ax_track.scatter(lon_traj, lat_traj, c=times, cmap='viridis', s=20, label='Trajectory', zorder=3)
            ax_track.plot(lon_traj[0], lat_traj[0], 'go', ms=12, label='Entry', zorder=5,
                         markeredgecolor='darkgreen', markeredgewidth=2)
            ax_track.plot(lon_traj[-1], lat_traj[-1], 'bs', ms=12, label='Deployment', zorder=5,
                         markeredgecolor='darkblue', markeredgewidth=2)
            ax_track.plot(lon_target, lat_target, 'r*', ms=20, label='Target', zorder=5,
                         markeredgecolor='darkred', markeredgewidth=1.5)
            ax_track.set_xlabel('Longitude [deg]', fontsize=11)
            ax_track.set_ylabel('Latitude [deg]', fontsize=11)
            ax_track.set_title('Best Trajectory Ground Track', fontsize=13, fontweight='bold')
            ax_track.legend(loc='best', fontsize=10)
            ax_track.grid(True, alpha=0.3)
            plt.colorbar(sc, ax=ax_track, label='Time [s]')
            plt.tight_layout()
            
            # Figure: State and control history (3x3 grid)
            fig_states, axes_states = plt.subplots(3, 3, figsize=(15, 12))
            axes_states = axes_states.flatten()
            
            # Altitude
            axes_states[0].plot(times, h_traj, 'b-', lw=2)
            axes_states[0].set_xlabel('Time [s]')
            axes_states[0].set_ylabel('Altitude [km]')
            axes_states[0].set_title('Altitude')
            axes_states[0].grid(True, alpha=0.3)
            
            # Velocity
            axes_states[1].plot(times, V_traj, 'r-', lw=2)
            axes_states[1].set_xlabel('Time [s]')
            axes_states[1].set_ylabel('Velocity [m/s]')
            axes_states[1].set_title('Velocity')
            axes_states[1].grid(True, alpha=0.3)
            
            # Flight path angle
            axes_states[2].plot(times, gam_traj, 'orange', lw=2)
            axes_states[2].set_xlabel('Time [s]')
            axes_states[2].set_ylabel('γ [deg]')
            axes_states[2].set_title('Flight Path Angle')
            axes_states[2].grid(True, alpha=0.3)
            
            # Heading angle
            axes_states[3].plot(times, psi_traj, 'brown', lw=2)
            axes_states[3].set_xlabel('Time [s]')
            axes_states[3].set_ylabel('ψ [deg]')
            axes_states[3].set_title('Heading Angle')
            axes_states[3].grid(True, alpha=0.3)
            
            # Bank angle
            axes_states[4].plot(times[:-1], sigma_traj, 'k-', lw=2)
            axes_states[4].axhline(120, ls='--', color='red', alpha=0.5)
            axes_states[4].axhline(-120, ls='--', color='red', alpha=0.5)
            axes_states[4].set_xlabel('Time [s]')
            axes_states[4].set_ylabel('σ [deg]')
            axes_states[4].set_title('Bank Angle (Logistic)')
            axes_states[4].grid(True, alpha=0.3)
            
            # Latitude
            axes_states[5].plot(times, lat_traj, 'm-', lw=2)
            axes_states[5].set_xlabel('Time [s]')
            axes_states[5].set_ylabel('Latitude [deg]')
            axes_states[5].set_title('Latitude')
            axes_states[5].grid(True, alpha=0.3)
            
            # Longitude
            axes_states[6].plot(times, lon_traj, 'g-', lw=2)
            axes_states[6].set_xlabel('Time [s]')
            axes_states[6].set_ylabel('Longitude [deg]')
            axes_states[6].set_title('Longitude')
            axes_states[6].grid(True, alpha=0.3)
            
            # Dynamic pressure
            axes_states[7].plot(times[1:], q_hist / 1e3, 'g-', lw=2)
            axes_states[7].axhline(constraints.q_max / 1e3, ls='--', color='red', alpha=0.5, label='Limit')
            axes_states[7].set_xlabel('Time [s]')
            axes_states[7].set_ylabel('q [kPa]')
            axes_states[7].set_title('Dynamic Pressure')
            axes_states[7].legend(fontsize=8)
            axes_states[7].grid(True, alpha=0.3)
            
            # Acceleration
            axes_states[8].plot(times[1:], A_hist, 'r-', lw=2)
            axes_states[8].axhline(constraints.A_max, ls='--', color='red', alpha=0.5, label='Limit')
            axes_states[8].set_xlabel('Time [s]')
            axes_states[8].set_ylabel('A [g]')
            axes_states[8].set_title('Acceleration')
            axes_states[8].legend(fontsize=8)
            axes_states[8].grid(True, alpha=0.3)
            
            plt.suptitle(f'Best Trajectory (Logistic): σ0={sigma0_best:.1f}°, K={K_best:.2f}', 
                        fontsize=14, fontweight='bold')
            plt.tight_layout()
            
            # Figure: Heat rate
            fig_heat, ax_heat = plt.subplots(figsize=(10, 5))
            ax_heat.plot(times[1:], Qdot_hist / 1e3, 'orange', lw=2, label='Heat Rate')
            ax_heat.axhline(constraints.Qdot_max / 1e3, ls='--', color='red', alpha=0.5, label='Limit')
            ax_heat.set_xlabel('Time [s]', fontsize=11)
            ax_heat.set_ylabel('Heat Rate [kW/m²]', fontsize=11)
            ax_heat.set_title('Heat Rate History', fontsize=13, fontweight='bold')
            ax_heat.legend(fontsize=10)
            ax_heat.grid(True, alpha=0.3)
            plt.tight_layout()
            
            # Figure: Mach number
            fig_mach, ax_mach = plt.subplots(figsize=(10, 5))
            ax_mach.plot(times[1:], mach_hist, 'purple', lw=2, label='Mach Number')
            ax_mach.axhline(1.4, ls='--', color='green', alpha=0.5, label='Deploy Min (M=1.4)')
            ax_mach.axhline(2.2, ls='--', color='blue', alpha=0.5, label='Deploy Max (M=2.2)')
            ax_mach.set_xlabel('Time [s]', fontsize=11)
            ax_mach.set_ylabel('Mach Number', fontsize=11)
            ax_mach.set_title('Mach Number History', fontsize=13, fontweight='bold')
            ax_mach.legend(fontsize=10)
            ax_mach.grid(True, alpha=0.3)
            plt.tight_layout()
            
        else:
            print("No valid targets found meeting all constraints!")

    else:
        print("No samples at all – check grid or initial conditions.")

    plt.show()
