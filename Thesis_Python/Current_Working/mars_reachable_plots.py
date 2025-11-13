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
    L_over_D = 0.24
    beta = 135.0            # ballistic coefficient as used in your eom
    N = 0.5                 # Sutton-Graves exponent on density
    M = 3.0                 # Sutton-Graves exponent on velocity
    k_heat_flux = 5.3697e-5 # Sutton-Graves coefficient


@dataclass
class PathConstraints:
    """Path constraints for the reentry vehicle (g-load, dynamic pressure, heat rate)."""
    A_max: float = 15.0        # [g] maximum total acceleration
    q_max: float = 13e3       # [Pa] maximum dynamic pressure
    Qdot_max: float = 500e3   # [W/m^2] maximum heat rate


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
# Simple bank profile: constant then linear ramp to 0
# ============================================================
def bank_profile(t, sigma_const=np.deg2rad(75), t1=40.0, t2=100.0):
    if t < t1:
        return sigma_const
    elif t < t2:
        # linear ramp to -20 degrees
        return sigma_const * (t2 - t) / (t2 - t1)
    else:
        return 0.0

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
    Original deploy rule based on Mach and dynamic pressure:

      - 1.4 <= Mach <= 2.2
      - 300 Pa <= q <= 800 Pa
      OR impact (h <= 0)
    """
    a = mars.sound_speed(h)
    a_safe = max(a, 1e-6)
    mach = V / a_safe

    return ((1.4 <= mach <= 2.2) and (300.0 <= q <= 800.0)) or (h <= 0.0)

# ============================================================
# Safe simulation with path constraints
# ============================================================
def simulate_entry_safe(initial_state,
                        dt=0.25,
                        tmax=2000.0,
                        sigma_const=np.deg2rad(75.0),
                        t1=40.0,
                        t2=100.0,
                        constraints=constraints):
    """
    Integrate one trajectory, tracking path constraint violations.

    Returns:
        state_f      : final state at stop time
        t_f          : final time
        violated     : True if any path constraint was violated
        max_q        : maximum dynamic pressure along the path [Pa]
        max_A        : maximum total acceleration along the path [g]
        max_Qdot     : maximum heat rate along the path [W/m^2]
    """
    state = initial_state.copy()
    t = 0.0

    max_q = 0.0
    max_A = 0.0
    max_Qdot = 0.0
    violated = False

    while t < tmax:
        sigma = bank_profile(t, sigma_const=sigma_const, t1=t1, t2=t2)
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
# Simulation with trace (for plotting one example trajectory)
# ============================================================
def simulate_entry_with_trace(initial_state,
                              dt=0.25,
                              tmax=2000.0,
                              sigma_const=np.deg2rad(75.0),
                              t1=40.0,
                              t2=100.0):
    """
    Simulate entry while recording state history and sigma(t).

    Returns:
        times         (N,)
        states        (N, 6)
        sigmas        (N,)
        trigger_index int or None
    """
    state = initial_state.copy()
    t = 0.0
    times = [t]
    states = [state.copy()]
    sigmas = [bank_profile(t, sigma_const=sigma_const, t1=t1, t2=t2)]
    trigger_index = None

    while t < tmax:
        sigma = bank_profile(t, sigma_const=sigma_const, t1=t1, t2=t2)
        state = rk4_step(eom, state, sigma, dt)
        t += dt

        times.append(t)
        states.append(state.copy())
        sigmas.append(sigma)

        # Compute loads to get q and Mach for deploy rule
        h, rho, q, A_total, Qdot = compute_loads(state)
        r, th, ph, V, gam, psi = state

        if deploy_trigger(h, V, q):
            trigger_index = len(times) - 1
            break

    return np.array(times), np.array(states), np.array(sigmas), trigger_index

# ============================================================
# Build SAFE / UNSAFE reachable set
# ============================================================
def build_reachable_set(lat0, lon0, lat_grid, lon_grid,
                        dt=0.25,
                        tmax=4000.0,
                        n_samples_per_cell=1):
    """
    Build a reachable set with all trajectory samples.

    Returns:
        all_samples : array of shape (N, 6) with columns
                      [RD_km, RC_km, ALT_km, max_q, max_A, max_Qdot]
    """
    all_samples = []

    # Fixed entry conditions
    h0 = 125e3
    r0 = mars.radius + h0
    V0 = 4700.0
    gam0 = np.deg2rad(-12.0)
    psi0 = np.deg2rad(-2.8758)

    for lat_t in lat_grid:
        for lon_t in lon_grid:
            for _ in range(n_samples_per_cell):
                state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])

                # Randomize bank-schedule times to explore reachable set
                t1 = random.uniform(0, 45)
                t2 = random.uniform(60, 150)

                state_f, tf, violated, max_q, max_A, max_Qdot = simulate_entry_safe(
                    state0, dt=dt, tmax=tmax, t1=t1, t2=t2
                )

                r, th, ph, V, gam, psi = state_f
                h = r - mars.radius

                # Metrics relative to the target (lat_t, lon_t)
                d_m, sin_d, cos_d, RC, RD, RD_go, Rgo = spherical_metrics(
                    lat0, lon0, ph, th, lat_t, lon_t, mars.radius
                )

                RD_km = RD / 1000.0
                RC_km = RC / 1000.0
                ALT_km = h / 1000.0

                record = [RD_km, RC_km, ALT_km, max_q, max_A, max_Qdot]
                all_samples.append(record)
                
                status = "VIOLATED" if violated else "OK"
                print(
                    f"[{status:8s}] target lat={np.degrees(lat_t):6.2f} deg, "
                    f"lon={np.degrees(lon_t):7.2f} deg | "
                    f"RD={RD_km:7.2f} km, RC={RC_km:7.2f} km, ALT={ALT_km:6.2f} km | "
                    f"max_q={max_q:8.1f} Pa, max_A={max_A:5.2f} g, max_Qdot={max_Qdot:8.1f} W/m^2"
                )

    all_samples = np.array(all_samples) if len(all_samples) > 0 else np.empty((0, 6))

    return all_samples

# ============================================================
# MAIN SCRIPT: grids, example trajectory, reachable set, plots
# ============================================================
if __name__ == "__main__":
    deg2rad = np.pi/180
    lat0 = -21.3 * deg2rad
    lon0 = -176.40167 * deg2rad

    # sweep ±30° latitude, ±2° longitude
    lat_grid = np.linspace(lat0 - 30*deg2rad, lat0 + 30*deg2rad, 10)
    lon_grid = np.linspace(lon0 - 2*deg2rad, lon0 + 2*deg2rad, 10)

    # ------------------------------------------------------------
    # Run one example trajectory (trace sigma and states) and plot
    # ------------------------------------------------------------
    # choose a sample target a few degrees away from the entry projection
    lat_t_example = lat0 + 15 * deg2rad
    lon_t_example = lon0 - 0.5 * deg2rad

    # initial state ordering: r, th(lon), ph(lat), V, gam, psi
    state0_example = np.array([mars.radius + 125e3,
                               lon0,
                               lat0,
                               4700.0,
                               np.deg2rad(-12.0),
                               np.deg2rad(-2.8758)])

    times, states, sigmas, trigger_idx = simulate_entry_with_trace(
        state0_example, dt=0.25, tmax=4000.0
    )
    if trigger_idx is None:
        trigger_idx = len(times) - 1

    state_trigger = states[trigger_idx]
    t_trigger = times[trigger_idx]
    sigma_trigger = sigmas[trigger_idx]

    print("Example trajectory deploy trigger at t = {:.2f} s".format(t_trigger))
    print("State at trigger (r, lon, lat, V, gam, psi):")
    print(state_trigger)
    print("Sigma at trigger: {:.4f} rad = {:.2f} deg".format(sigma_trigger, np.degrees(sigma_trigger)))

    # Lat/Lon plot for the trajectory
    th_array = states[:,1]
    ph_array = states[:,2]
    plt.figure(figsize=(8,5))
    plt.plot(np.degrees(th_array), np.degrees(ph_array), '-o', markersize=3, label='trajectory')
    plt.scatter(np.degrees(lon0), np.degrees(lat0), color='green', s=60, label='initial (entry proj)')
    plt.scatter(np.degrees(lon_t_example), np.degrees(lat_t_example), color='red', s=60, label='target')
    plt.scatter(np.degrees(state_trigger[1]), np.degrees(state_trigger[2]),
                color='orange', s=80, marker='*', label='deploy trigger')
    plt.xlabel('Longitude [deg]')
    plt.ylabel('Latitude [deg]')
    plt.title('Example trajectory: lon/lat (initial, target, deploy)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Sigma vs time
    plt.figure(figsize=(8,3))
    plt.plot(times, np.degrees(sigmas), '-k')
    plt.axvline(t_trigger, color='orange', linestyle='--', label='deploy trigger')
    plt.xlabel('Time [s]')
    plt.ylabel('Bank angle sigma [deg]')
    plt.title('Bank profile sigma(t) for example trajectory')
    plt.legend()
    plt.tight_layout()

    # ------------------------------------------------------------
    # Combined figure: all states vs time (single figure with subplots)
    # ------------------------------------------------------------
    h_array = states[:,0] - mars.radius
    r_array = states[:,0]
    lon_array = np.degrees(states[:,1])
    lat_array = np.degrees(states[:,2])
    V_array = states[:,3]
    gam_array = np.degrees(states[:,4])
    psi_array = np.degrees(states[:,5])
    sigma_deg = np.degrees(sigmas)

    fig, axes = plt.subplots(4, 2, figsize=(12, 10))
    axes = axes.flatten()

    axes[0].plot(times, h_array/1000.0, '-b')
    axes[0].set_ylabel('Altitude [km]')
    axes[0].set_title('Altitude (h)')

    axes[1].plot(times, r_array/1000.0, '-c')
    axes[1].set_ylabel('Radius [km]')

    axes[2].plot(times, lon_array, '-g')
    axes[2].set_ylabel('Longitude [deg]')

    axes[3].plot(times, lat_array, '-m')
    axes[3].set_ylabel('Latitude [deg]')

    axes[4].plot(times, V_array, '-k')
    axes[4].set_ylabel('Velocity [m/s]')

    axes[5].plot(times, gam_array, '-r')
    axes[5].set_ylabel('Flight path angle γ [deg]')

    axes[6].plot(times, psi_array, '-')
    axes[6].set_ylabel('Heading ψ [deg]')

    axes[7].plot(times, sigma_deg)
    axes[7].axvline(t_trigger, color='orange', linestyle='--')
    axes[7].set_ylabel('Bank σ [deg]')

    for ax in axes:
        ax.set_xlabel('Time [s]')
        ax.grid(True)

    plt.suptitle('All example trajectory states vs time', y=1.02)
    plt.tight_layout()

    # ------------------------------------------------------------
    # Build reachable set
    # ------------------------------------------------------------
    all_samples = build_reachable_set(
        lat0, lon0, lat_grid, lon_grid,
        dt=0.25, tmax=4000.0,
        n_samples_per_cell=1
    )

    # ------------------------------------------------------------
    # Path constraint reachable set: plot actual max values
    # ------------------------------------------------------------
    if all_samples.shape[0] > 0:
        RD_all   = all_samples[:, 0]
        RC_all   = all_samples[:, 1]
        ALT_all  = all_samples[:, 2]
        q_all    = all_samples[:, 3]  # max dynamic pressure [Pa]
        A_all    = all_samples[:, 4]  # max acceleration [g]
        Qdot_all = all_samples[:, 5]  # max heat rate [W/m^2]

        # Create 2x2 subplot: altitude + 3 path constraints
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 1) Altitude contour
        levels_alt = 12
        cf0 = axes[0,0].tricontourf(RD_all, RC_all, ALT_all, levels=levels_alt, cmap='viridis')
        cs0 = axes[0,0].tricontour(RD_all, RC_all, ALT_all, levels=levels_alt, colors='k', linewidths=0.6)
        axes[0,0].clabel(cs0, fmt="%.1f", fontsize=8)
        cbar0 = fig.colorbar(cf0, ax=axes[0,0], label='Altitude [km]')
        axes[0,0].set_xlabel('Downrange [km]')
        axes[0,0].set_ylabel('Crossrange [km]')
        axes[0,0].set_title('Deployment altitude')
        axes[0,0].grid(True)

        # 2) Maximum dynamic pressure
        levels_q = 12
        q_kPa = q_all / 1e3  # convert Pa to kPa
        cf1 = axes[0,1].tricontourf(RD_all, RC_all, q_kPa, levels=levels_q, cmap='plasma')
        cs1 = axes[0,1].tricontour(RD_all, RC_all, q_kPa, levels=levels_q, colors='k', linewidths=0.6)
        axes[0,1].clabel(cs1, fmt="%.1f", fontsize=8)
        cbar1 = fig.colorbar(cf1, ax=axes[0,1], label='Max q [kPa]')
        axes[0,1].axhline(y=0, color='white', linestyle='--', linewidth=2, alpha=0.5)
        axes[0,1].text(0.02, 0.98, f'Limit: {constraints.q_max/1e3:.1f} kPa', 
                      transform=axes[0,1].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes[0,1].set_xlabel('Downrange [km]')
        axes[0,1].set_ylabel('Crossrange [km]')
        axes[0,1].set_title('Maximum dynamic pressure')
        axes[0,1].grid(True)

        # 3) Maximum acceleration
        levels_a = 12
        cf2 = axes[1,0].tricontourf(RD_all, RC_all, A_all, levels=levels_a, cmap='hot')
        cs2 = axes[1,0].tricontour(RD_all, RC_all, A_all, levels=levels_a, colors='k', linewidths=0.6)
        axes[1,0].clabel(cs2, fmt="%.2f", fontsize=8)
        cbar2 = fig.colorbar(cf2, ax=axes[1,0], label='Max A [g]')
        axes[1,0].text(0.02, 0.98, f'Limit: {constraints.A_max:.1f} g', 
                      transform=axes[1,0].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes[1,0].set_xlabel('Downrange [km]')
        axes[1,0].set_ylabel('Crossrange [km]')
        axes[1,0].set_title('Maximum acceleration')
        axes[1,0].grid(True)

        # 4) Maximum heat rate
        levels_qd = 12
        Qdot_kW = Qdot_all / 1e3  # convert W/m^2 to kW/m^2
        cf3 = axes[1,1].tricontourf(RD_all, RC_all, Qdot_kW, levels=levels_qd, cmap='inferno')
        cs3 = axes[1,1].tricontour(RD_all, RC_all, Qdot_kW, levels=levels_qd, colors='k', linewidths=0.6)
        axes[1,1].clabel(cs3, fmt="%.1f", fontsize=8)
        cbar3 = fig.colorbar(cf3, ax=axes[1,1], label='Max Q̇ [kW/m²]')
        axes[1,1].text(0.02, 0.98, f'Limit: {constraints.Qdot_max/1e3:.0f} kW/m²', 
                      transform=axes[1,1].transAxes, fontsize=10, color='white',
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        axes[1,1].set_xlabel('Downrange [km]')
        axes[1,1].set_ylabel('Crossrange [km]')
        axes[1,1].set_title('Maximum heat rate')
        axes[1,1].grid(True)

        plt.suptitle('Reachable Set: Maximum Path Constraint Values', fontsize=14, y=0.995)
        plt.tight_layout()

    else:
        print("No samples at all – check grid or initial conditions.")

    plt.show()






