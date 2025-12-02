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
# Three-section bank profile parameterization
# ============================================================
def bank_profile_three_section(t, t_total, sigma1, sigma2):
    """
    Three-section bank angle profile:
    - Section 1 (0% to 75% of t_total): constant bank angle sigma1
    - Section 2 (75% to 77% of t_total): linear transition from sigma1 to sigma2
    - Section 3 (77% to 100% of t_total): constant bank angle sigma2
    
    Args:
        t: current time [s]
        t_total: total trajectory duration [s]
        sigma1: bank angle for first section [rad], in range [-81°, +81°]
        sigma2: bank angle for third section [rad], in range [-81°, +81°]
    
    Returns:
        bank angle [rad]
    """
    t1 = 0.75 * t_total  # End of section 1 (35%)
    t2 = 0.77 * t_total  # End of section 2 (50%)
    
    if t < t1:
        # Section 1: constant sigma1
        return sigma1
    elif t < t2:
        # Section 2: linear transition from sigma1 to sigma2
        alpha = (t - t1) / (t2 - t1)  # interpolation factor [0, 1]
        return sigma1 + alpha * (sigma2 - sigma1)
    else:
        # Section 3: constant sigma2
        return sigma2

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
# Safe simulation with path constraints
# ============================================================
def simulate_entry_safe(initial_state,
                        dt=0.25,
                        tmax=2000.0,
                        sigma1=np.deg2rad(75.0),
                        sigma2=np.deg2rad(-75.0),
                        constraints=constraints):
    """
    Integrate one trajectory using three-section bank profile.
    
    Uses two-pass approach:
    1. First pass: estimate actual trajectory duration
    2. Second pass: re-simulate with bank profile normalized to actual duration

    Returns:
        state_f      : final state at stop time
        t_f          : final time
        violated     : True if any path constraint was violated
        max_q        : maximum dynamic pressure along the path [Pa]
        max_A        : maximum total acceleration along the path [g]
        max_Qdot     : maximum heat rate along the path [W/m^2]
    """
    # First pass: estimate trajectory duration using tmax as duration guess
    state = initial_state.copy()
    t = 0.0

    while t < tmax:
        sigma = bank_profile_three_section(t, tmax, sigma1, sigma2)
        state = rk4_step(eom, state, sigma, dt)
        h, rho, q, A_total, Qdot = compute_loads(state)
        r, th, ph, V, gam, psi = state
        
        if deploy_trigger(h, V, q):
            break
        t += dt
    
    t_actual = t  # Actual trajectory duration
    
    # Second pass: re-simulate with bank profile normalized to actual duration
    state = initial_state.copy()
    t = 0.0

    max_q = 0.0
    max_A = 0.0
    max_Qdot = 0.0
    violated = False

    while t < tmax:  # Slightly beyond to ensure we reach deployment
        sigma = bank_profile_three_section(t, t_actual, sigma1, sigma2)
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
                        sigma1_grid, sigma2_grid,
                        dt=0.25,
                        tmax=4000.0):
    """
    Build a reachable set by sweeping control parameters (sigma1, sigma2).

    Args:
        lat0, lon0: entry point latitude and longitude [rad]
        sigma1_grid: array of sigma1 values to sweep [rad]
        sigma2_grid: array of sigma2 values to sweep [rad]
        dt: integration timestep [s]
        tmax: maximum simulation time [s]

    Returns:
        all_samples : array of shape (N, 9) with columns
                      [lat_f_deg, lon_f_deg, ALT_km, max_q, max_A, max_Qdot, sigma1_deg, sigma2_deg, gam_f_deg]
    """
    all_samples = []

    # Fixed entry conditions
    h0 = 128e3
    r0 = mars.radius + h0
    V0 = 5500.0
    gam0 = np.deg2rad(-15.5)
    psi0 = np.deg2rad(90.0)

    state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])

    for sigma1 in sigma1_grid:
        for sigma2 in sigma2_grid:
            state_f, tf, violated, max_q, max_A, max_Qdot = simulate_entry_safe(
                state0, dt=dt, tmax=tmax, sigma1=sigma1, sigma2=sigma2
            )

            r, th, ph, V, gam, psi = state_f
            h = r - mars.radius

            lat_f_deg = np.degrees(ph)
            lon_f_deg = np.degrees(th)
            ALT_km = h / 1000.0
            gam_f_deg = np.degrees(gam)
            sigma1_deg = np.degrees(sigma1)
            sigma2_deg = np.degrees(sigma2)

            record = [lat_f_deg, lon_f_deg, ALT_km, max_q, max_A, max_Qdot, sigma1_deg, sigma2_deg, gam_f_deg]
            all_samples.append(record)
            
            status = "VIOLATED" if violated else "OK"
            print(
                f"[{status:8s}] σ1={sigma1_deg:6.2f}°, σ2={sigma2_deg:6.2f}° | "
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

    # Sweep control parameters: sigma1 and sigma2 in range [-81°, +81°]
    # Coarse mesh for speed (10x10 = 100 simulations)
    sigma1_grid = np.linspace(np.deg2rad(-81), np.deg2rad(81), 10)
    sigma2_grid = np.linspace(np.deg2rad(-81), np.deg2rad(81), 10)

    print(f"Building reachable set with {len(sigma1_grid)} x {len(sigma2_grid)} = {len(sigma1_grid)*len(sigma2_grid)} control parameter combinations")
    print(f"sigma1 range: [{np.degrees(sigma1_grid[0]):.1f}°, {np.degrees(sigma1_grid[-1]):.1f}°]")
    print(f"sigma2 range: [{np.degrees(sigma2_grid[0]):.1f}°, {np.degrees(sigma2_grid[-1]):.1f}°]")
    print("="*80)

    # ------------------------------------------------------------
    # Build reachable set
    # ------------------------------------------------------------
    all_samples = build_reachable_set(
        lat0, lon0, sigma1_grid, sigma2_grid,
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
            d_m, sin_d, cos_d, RC, RD, RD_go, Rgo = spherical_metrics(
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

        plt.suptitle('Reachable Set: Lat/Lon Footprint', fontsize=14, y=0.995)
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

        plt.suptitle('Reachable Set: Downrange/Crossrange Footprint', fontsize=14, y=0.995)
        plt.tight_layout()

        # ============================================================
        # TARGET SELECTION: Apply cost function to choose best target
        # ============================================================
        print("\n" + "="*80)
        print("TARGET SELECTION")
        print("="*80)
        
        # Apply constraints to filter valid targets
        valid_mask = (
            (np.abs(RD_all) < 10.0) &          # Crossrange < 10 km
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
            sigma1_valid = all_samples[valid_mask, 6]
            sigma2_valid = all_samples[valid_mask, 7]
            
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
            w_alt = 0.4      # Weight for altitude (higher is better)
            w_A = 0.05       # Weight for acceleration (lower is better)
            w_Qdot = 0.1     # Weight for heat rate (lower is better)
            w_RC = 0.15      # Weight for crossrange (lower is better)
            w_gam = 0.3      # Weight for flight path angle (lower is better)
            
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
            sigma1_best = sigma1_valid[best_idx]
            sigma2_best = sigma2_valid[best_idx]
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
            print(f"  Control: σ1={sigma1_best:6.2f}°, σ2={sigma2_best:6.2f}°")
            
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
            
            # First pass: estimate trajectory duration
            dt_sim = 0.25
            tmax_sim = 4000.0
            sigma1_rad = np.deg2rad(sigma1_best)
            sigma2_rad = np.deg2rad(sigma2_best)
            
            state = state0.copy()
            t = 0.0
            
            while t < tmax_sim:
                sigma = bank_profile_three_section(t, tmax_sim, sigma1_rad, sigma2_rad)
                state = rk4_step(eom, state, sigma, dt_sim)
                h, rho, q, A_total, Qdot = compute_loads(state)
                r, th, ph, V, gam, psi = state
                
                if deploy_trigger(h, V, q):
                    break
                t += dt_sim
            
            t_actual = t
            print(f"Estimated trajectory duration: {t_actual:.2f} s")
            
            # Second pass: simulate with bank profile normalized to actual duration
            state = state0.copy()
            t = 0.0
            times = [t]
            states = [state.copy()]
            sigmas = [bank_profile_three_section(t, t_actual, sigma1_rad, sigma2_rad)]
            q_hist = []
            A_hist = []
            Qdot_hist = []
            mach_hist = []
            
            while t < tmax_sim:
                sigma = bank_profile_three_section(t, t_actual, sigma1_rad, sigma2_rad)
                state = rk4_step(eom, state, sigma, dt_sim)
                t += dt_sim
                
                times.append(t)
                states.append(state.copy())
                sigmas.append(sigma)
                
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
            axes_states[4].plot(times, sigma_traj, 'k-', lw=2)
            axes_states[4].axhline(81, ls='--', color='red', alpha=0.5)
            axes_states[4].axhline(-81, ls='--', color='red', alpha=0.5)
            axes_states[4].set_xlabel('Time [s]')
            axes_states[4].set_ylabel('σ [deg]')
            axes_states[4].set_title('Bank Angle')
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
            
            plt.suptitle(f'Best Trajectory: σ1={sigma1_best:.1f}°, σ2={sigma2_best:.1f}°', 
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






