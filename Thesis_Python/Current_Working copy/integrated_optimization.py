"""
Integrated Optimization Script
================================
This script combines the reachable set analysis with direct collocation optimization:

1. Run mars_reachable_plots to find best feasible target
2. Use best target to constrain collocation optimization
3. Initialize collocation with control parameters from reachable set
4. Plot comprehensive results including:
   - Reachable set footprints (lat/lon and downrange/crossrange)
   - Optimized trajectory time histories
   - Ground track comparison
   - Deployment envelope
   - Path constraints verification
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import casadi as ca
import time
from spherical_metrics import spherical_metrics

# ============================================================
# MODELS & CONSTANTS (shared between both methods)
# ============================================================

@dataclass
class Mars:
    """Mars atmospheric and gravitational model."""
    radius: float = 3396.2e3
    mu: float = 4.282837e13
    rho0: float = 0.02
    Hs: float = 11.1e3
    omega: float = 7.088e-5
    gamma_gas: float = 1.3
    R_gas: float = 191.0
    
    def sound_speed(self, h):
        h_km = h / 1e3
        T = 1.4e-13 * h_km**3 - 8.85e-9 * h_km**2 - 1.245e-3 * h_km + 205.36
        return np.sqrt(self.gamma_gas * self.R_gas * max(T, 1.0))

@dataclass
class Vehicle:
    """Entry vehicle properties."""
    beta: float = 135.0
    L_over_D: float = 0.24
    k_heat_flux: float = 5.3697e-5
    N: float = 0.5
    M: float = 3.15

@dataclass
class Constraints:
    """Path constraints."""
    q_max: float = 13000.0
    A_max: float = 15.0
    Qdot_max: float = 500000.0
    
@dataclass
class DeployRegion:
    """Parachute deployment envelope (single source of truth)."""
    mach_min: float = 1.4
    mach_max: float = 2.2
    q_min: float = 300.0      # Pa
    q_max: float = 800.0      # Pa
    h_min: float = 6e3        # m (minimum deployment altitude)

deploy_region = DeployRegion()


mars = Mars()
veh = Vehicle()
constraints = Constraints()

# Entry conditions
deg = np.deg2rad
h0 = 125e3
r0 = mars.radius + h0
V0_reachable = 5000.0  # For reachable set
V0_collocation = 5000.0  # For collocation
gam0 = deg(-12.0)
psi0 = deg(-2.8758)
lat0 = deg(-21.5)
lon0 = deg(-176.40167)

# Control limits
SIGMA_MIN = deg(-81.0)
SIGMA_MAX = deg(81.0)
SIGMA_DOT_MAX = deg(20.0)
SIGMA_DOT_MIN = -deg(20.0)
SIGMA_DOT_DOT_MAX = deg(5.0)
SIGMA_DOT_DOT_MIN = -deg(5.0)

# ============================================================
# REACHABLE SET FUNCTIONS (from mars_reachable_plots.py)
# ============================================================

def bank_profile_three_section(t, t_total, sigma1, sigma2):
    """Three-section bank angle profile."""
    t1 = 0.35 * t_total
    t2 = 0.50 * t_total
    
    if t < t1:
        return sigma1
    elif t < t2:
        return sigma1 * (t2 - t) / (t2 - t1)
    else:
        return sigma2

def eom_numpy(state, sigma):
    """3DOF equations of motion (NumPy version)."""
    r, th, ph, V, gam, psi = state
    
    g = mars.mu / r**2
    rho = mars.rho0 * np.exp(-(r - mars.radius) / mars.Hs)
    D = 0.5 * rho * V**2 / max(veh.beta, 1e-9)
    L = veh.L_over_D * D
    Vh = V * np.cos(gam)
    
    r_dot = V * np.sin(gam)
    th_dot = (Vh * np.sin(psi)) / (r * max(np.cos(ph), 1e-6))
    ph_dot = (Vh * np.cos(psi)) / r
    V_dot = -D - g * np.sin(gam) + mars.omega**2 * r * np.cos(ph) * \
            (np.sin(gam)*np.cos(ph) - np.cos(gam)*np.sin(ph)*np.cos(psi))
    gam_dot = (L*np.cos(sigma)/max(V,1e-6)) + (V/r - g/max(V,1e-6))*np.cos(gam) + \
              2*mars.omega*np.cos(ph)*np.sin(psi) + \
              (mars.omega**2*r/max(V,1e-6))*np.cos(ph)* \
              (np.cos(gam)*np.cos(ph) + np.sin(gam)*np.sin(ph)*np.cos(psi))
    psi_dot = (L*np.sin(sigma))/(max(V,1e-6)*max(np.cos(gam),1e-6)) + \
              (Vh/r)*np.sin(psi)*np.tan(ph) - \
              2*mars.omega*(np.tan(gam)*np.cos(ph)*np.cos(psi) - np.sin(ph)) + \
              (mars.omega**2*r/(max(V,1e-6)*max(np.cos(gam),1e-6)))*np.sin(ph)*np.cos(ph)*np.sin(psi)
    
    return np.array([r_dot, th_dot, ph_dot, V_dot, gam_dot, psi_dot])

def rk4_step(f, state, sigma, dt):
    """RK4 integration step."""
    k1 = f(state, sigma)
    k2 = f(state + 0.5*dt*k1, sigma)
    k3 = f(state + 0.5*dt*k2, sigma)
    k4 = f(state + dt*k3, sigma)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def compute_loads(state):
    """Compute aerodynamic loads."""
    r, th, ph, V, gam, psi = state
    h = r - mars.radius
    rho = mars.rho0 * np.exp(-h / mars.Hs)
    
    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D
    q = 0.5 * rho * V**2
    
    g0 = 9.80665
    A_total = np.sqrt(D**2 + L**2) / g0
    
    Qdot = veh.k_heat_flux * (rho**veh.N) * (V**veh.M)
    
    return h, rho, q, A_total, Qdot

def deploy_trigger(h, V, q, mars=mars, region=deploy_region):
    """Deployment trigger based on unified Mach/q/altitude envelope."""
    a = mars.sound_speed(h)
    a_safe = max(a, 1e-6)
    mach = V / a_safe

    in_box = (
        (region.mach_min <= mach <= region.mach_max) and
        (region.q_min    <= q    <= region.q_max) and
        (h >= region.h_min)
    )

    # h <= 0 is still a hard stop in the integrator
    return in_box or (h <= 6.0e3)


def simulate_entry_safe(initial_state, dt=0.25, tmax=2000.0,
                        sigma1=np.deg2rad(75.0), sigma2=np.deg2rad(-75.0),
                        constraints=constraints):
    """Simulate entry with two-pass approach for correct bank profile timing."""
    # First pass: estimate trajectory duration
    state = initial_state.copy()
    t = 0.0
    
    while t < tmax:
        sigma = bank_profile_three_section(t, tmax, sigma1, sigma2)
        state = rk4_step(eom_numpy, state, sigma, dt)
        h, rho, q, A_total, Qdot = compute_loads(state)
        r, th, ph, V, gam, psi = state
        
        if deploy_trigger(h, V, q):
            break
        t += dt
    
    t_actual = t
    
    # Second pass: re-simulate with normalized bank profile
    state = initial_state.copy()
    t = 0.0
    
    max_q = 0.0
    max_A = 0.0
    max_Qdot = 0.0
    violated = False
    
    while t < tmax:
        sigma = bank_profile_three_section(t, t_actual, sigma1, sigma2)
        state = rk4_step(eom_numpy, state, sigma, dt)
        
        h, rho, q, A_total, Qdot = compute_loads(state)
        r, th, ph, V, gam, psi = state
        
        max_q = max(max_q, q)
        max_A = max(max_A, A_total)
        max_Qdot = max(max_Qdot, Qdot)
        
        if (q > constraints.q_max or
            A_total > constraints.A_max or
            Qdot > constraints.Qdot_max):
            violated = True
        
        if deploy_trigger(h, V, q):
            break
        
        t += dt
    
    return state, t, violated, max_q, max_A, max_Qdot

def build_reachable_set(lat0, lon0, sigma1_grid, sigma2_grid,
                        dt=0.25, tmax=4000.0):
    """Build reachable set by sweeping control parameters."""
    print("\n" + "="*80)
    print("BUILDING REACHABLE SET")
    print("="*80)
    print(f"Control grid: {len(sigma1_grid)} × {len(sigma2_grid)} = {len(sigma1_grid)*len(sigma2_grid)} samples")
    
    h0 = 125e3
    r0 = mars.radius + h0
    V0 = V0_reachable
    gam0 = np.deg2rad(-12.0)
    psi0 = np.deg2rad(-2.8758)
    state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])
    
    all_samples = []
    
    for i, s1 in enumerate(sigma1_grid):
        for j, s2 in enumerate(sigma2_grid):
            state_f, t_f, violated, max_q, max_A, max_Qdot = simulate_entry_safe(
                state0, dt=dt, tmax=tmax, sigma1=s1, sigma2=s2, constraints=constraints
            )
            
            r_f, lon_f, lat_f, V_f, gam_f, psi_f = state_f
            h_f = r_f - mars.radius
            
            all_samples.append([
                lat_f, lon_f, h_f/1e3,  # lat, lon, altitude [km]
                max_q, max_A, max_Qdot,  # max loads
                np.rad2deg(s1), np.rad2deg(s2),  # control parameters [deg]
                t_f  # trajectory duration [s]
            ])
            #print(f"Sample ({i+1},{j+1}): h_final={h_f/1e3:7.2f} km, max_q={max_q:8.1f} Pa, max_A={max_A:5.2f} g, max_Qdot={max_Qdot:8.1f} W/m^2, violated={violated}")
    
    all_samples = np.array(all_samples)
    print(f"Generated {len(all_samples)} samples")
    
    return all_samples

def find_best_target(all_samples, lat0, lon0):
    """Find best target using cost function."""
    print("\n" + "="*80)
    print("TARGET SELECTION")
    print("="*80)
    
    # Extract data
    lat_all = all_samples[:, 0]
    lon_all = all_samples[:, 1]
    ALT_all = all_samples[:, 2]
    q_all = all_samples[:, 3]
    A_all = all_samples[:, 4]
    Qdot_all = all_samples[:, 5]
    sigma1_all = all_samples[:, 6]
    sigma2_all = all_samples[:, 7]
    t_all = all_samples[:, 8]
    
    # Compute downrange/crossrange
    lat_ref = lat0 + np.deg2rad(1.0)
    lon_ref = lon0
    
    RD_all = np.zeros(len(lat_all))
    RC_all = np.zeros(len(lat_all))
    
    for i in range(len(lat_all)):
        _, _, _, RC_all[i], RD_all[i], _, _ = spherical_metrics(
            lat0, lon0, lat_all[i], lon_all[i],
            lat_ref, lon_ref, mars.radius
        )
    
    RD_all /= 1e3
    RC_all /= 1e3
    
    # Apply constraints
    valid_mask = (
        (np.abs(RC_all) < 10.0) &
        (A_all < constraints.A_max) &
        (q_all <= constraints.q_max) &
        (Qdot_all <= constraints.Qdot_max)
    )
    
    n_valid = np.sum(valid_mask)
    print(f"Valid targets: {n_valid} / {len(lat_all)}")
    
    if n_valid == 0:
        print("ERROR: No valid targets found!")
        return None
    
    # Extract valid samples
    lat_valid = lat_all[valid_mask]
    lon_valid = lon_all[valid_mask]
    ALT_valid = ALT_all[valid_mask]
    RC_valid = RC_all[valid_mask]
    RD_valid = RD_all[valid_mask]
    q_valid = q_all[valid_mask]
    A_valid = A_all[valid_mask]
    Qdot_valid = Qdot_all[valid_mask]
    sigma1_valid = sigma1_all[valid_mask]
    sigma2_valid = sigma2_all[valid_mask]
    t_valid = t_all[valid_mask]
    
    # Cost function: maximize altitude
    cost = -ALT_valid
    best_idx = np.argmin(cost)
    
    best_target = {
        'lat': lat_valid[best_idx],
        'lon': lon_valid[best_idx],
        'altitude': ALT_valid[best_idx],
        'downrange': RD_valid[best_idx],
        'crossrange': RC_valid[best_idx],
        'max_q': q_valid[best_idx],
        'max_A': A_valid[best_idx],
        'max_Qdot': Qdot_valid[best_idx],
        'sigma1': sigma1_valid[best_idx],
        'sigma2': sigma2_valid[best_idx],
        't_final': t_valid[best_idx]
    }
    
    print(f"\nBest target selected:")
    print(f"  Latitude:     {np.rad2deg(best_target['lat']):7.3f}°")
    print(f"  Longitude:    {np.rad2deg(best_target['lon']):7.3f}°")
    print(f"  Downrange:    {best_target['downrange']:7.2f} km")
    print(f"  Crossrange:   {best_target['crossrange']:7.2f} km")
    print(f"  Altitude:     {best_target['altitude']:7.2f} km")
    print(f"  Max q:        {best_target['max_q']:8.1f} Pa")
    print(f"  Max A:        {best_target['max_A']:5.2f} g")
    print(f"  Max Qdot:     {best_target['max_Qdot']:8.1f} W/m²")
    print(f"  Control: σ1={best_target['sigma1']:6.2f}°, σ2={best_target['sigma2']:6.2f}°")
    print(f"  Time:         {best_target['t_final']:.1f} s")
    
    return best_target, all_samples, RD_all, RC_all

# ============================================================
# COLLOCATION OPTIMIZATION (from Collocation_Method.py)
# ============================================================

def eom_casadi(state, sigma):
    """CasADi symbolic version of equations of motion."""
    r, lon, lat, V, gam, psi = state[0], state[1], state[2], state[3], state[4], state[5]
    
    g = mars.mu / r**2
    h = r - mars.radius
    rho = mars.rho0 * ca.exp(-h / mars.Hs)
    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D
    Vh = V * ca.cos(gam)
    
    r_dot = V * ca.sin(gam)
    lon_dot = (Vh * ca.sin(psi)) / (r * ca.fmax(ca.cos(lat), 1e-6))
    lat_dot = (Vh * ca.cos(psi)) / r
    V_dot = -D - g * ca.sin(gam) + mars.omega**2 * r * ca.cos(lat) * \
            (ca.sin(gam)*ca.cos(lat) - ca.cos(gam)*ca.sin(lat)*ca.cos(psi))
    gam_dot = (L*ca.cos(sigma)/ca.fmax(V,1e-6)) + (V/r - g/ca.fmax(V,1e-6))*ca.cos(gam) + \
              2*mars.omega*ca.cos(lat)*ca.sin(psi) + \
              (mars.omega**2*r/ca.fmax(V,1e-6))*ca.cos(lat)* \
              (ca.cos(gam)*ca.cos(lat) + ca.sin(gam)*ca.sin(lat)*ca.cos(psi))
    psi_dot = (L*ca.sin(sigma))/(ca.fmax(V,1e-6)*ca.fmax(ca.cos(gam),1e-6)) + \
              (Vh/r)*ca.sin(psi)*ca.tan(lat) - \
              2*mars.omega*(ca.tan(gam)*ca.cos(lat)*ca.cos(psi) - ca.sin(lat)) + \
              (mars.omega**2*r/(ca.fmax(V,1e-6)*ca.fmax(ca.cos(gam),1e-6)))*ca.sin(lat)*ca.cos(lat)*ca.sin(psi)
    
    return ca.vertcat(r_dot, lon_dot, lat_dot, V_dot, gam_dot, psi_dot)

def solve_collocation_with_target(best_target):
    """Solve collocation optimization with target from reachable set."""
    
    print("\n" + "="*80)
    print("DIRECT COLLOCATION OPTIMIZATION")
    print("="*80)
    
    N_SEGMENTS = 200
    lat_target = best_target['lat']
    lon_target = best_target['lon']
    
    state0 = np.array([r0, lon0, lat0, V0_collocation, gam0, psi0])
    
    print(f"Target: lat={np.rad2deg(lat_target):.3f}°, lon={np.rad2deg(lon_target):.3f}°")
    print(f"Control initialization: σ1={best_target['sigma1']:.1f}°, σ2={best_target['sigma2']:.1f}°")
    
    # Create optimization problem
    opti = ca.Opti()
    
    X = opti.variable(6, N_SEGMENTS + 1)
    U = opti.variable(1, N_SEGMENTS)
    dt = opti.variable()
    
    r = X[0, :]
    lon = X[1, :]
    lat = X[2, :]
    V = X[3, :]
    gam = X[4, :]
    psi = X[5, :]
    sigma = U[0, :]
    
    # Objective: maximize altitude while minimizing gamma^2
    # Objective weights (rescaled)
    W_ALTITUDE = 1e-3     # h in meters → scale down
    W_GAMMA    = 1.0
    W_DEPLOY   = 10.0     # how strongly we prefer the middle of the deploy box

    h_final = r[-1] - mars.radius
    gam_final = gam[-1]
    V_f = V[-1]

    # Terminal Mach/q again
    rho_f = mars.rho0 * ca.exp(-h_final / mars.Hs)
    q_f   = 0.5 * rho_f * V_f**2

    h_km_f = h_final / 1e3
    T_f = 1.4e-13 * h_km_f**3 - 8.85e-9 * h_km_f**2 - 1.245e-3 * h_km_f + 205.36
    a_sound_f = ca.sqrt(mars.gamma_gas * mars.R_gas * ca.fmax(T_f, 1.0))
    mach_f = V_f / ca.fmax(a_sound_f, 1e-6)
    MACH_MIN = deploy_region.mach_min
    MACH_MAX = deploy_region.mach_max
    Q_MIN    = deploy_region.q_min
    Q_MAX    = deploy_region.q_max
    H_MIN    = deploy_region.h_min

    # Center of deploy box (Mach and q)
    mach_c = 0.5 * (MACH_MIN + MACH_MAX)
    q_c    = 0.5 * (Q_MIN    + Q_MAX)

    # Normalized offsets from center
    mach_norm = (mach_f - mach_c) / (0.5 * (MACH_MAX - MACH_MIN) + 1e-6)
    q_norm    = (q_f    - q_c   ) / (0.5 * (Q_MAX    - Q_MIN   ) + 1e-6)

    objective = 0
    # maximize altitude
    objective += -W_ALTITUDE * h_final
    # penalize very steep final gamma (optional; you can turn this off if you want)
    objective += W_GAMMA * gam_final**2
    # keep deployment near the middle of the Mach / q box
    objective += W_DEPLOY * (mach_norm**2 + q_norm**2)

    opti.minimize(objective)
    
    # Boundary conditions
    opti.subject_to(X[:, 0] == state0)
    
    # Terminal position constraint: hit the target
    lat_f = lat[-1]
    lon_f = lon[-1]
    dlat = lat_f - lat_target
    dlon = lon_f - lon_target
    a = ca.sin(dlat/2)**2 + ca.cos(lat_f) * ca.cos(lat_target) * ca.sin(dlon/2)**2
    c = 2 * ca.atan2(ca.sqrt(a), ca.sqrt(1-a))
    miss_distance = mars.radius * c
    
    # Hard constraint: miss distance must be less than 10 km
    opti.subject_to(miss_distance <= 5000.0)
    
    # Dynamics constraints (Hermite-Simpson)
    for k in range(N_SEGMENTS):
        X_k = X[:, k]
        X_kp1 = X[:, k+1]
        U_k = U[:, k]
        
        X_mid = 0.5 * (X_k + X_kp1) + (dt/8.0) * (eom_casadi(X_k, U_k) - eom_casadi(X_kp1, U_k))
        
        f_k = eom_casadi(X_k, U_k)
        f_mid = eom_casadi(X_mid, U_k)
        f_kp1 = eom_casadi(X_kp1, U_k)
        
        opti.subject_to(X_kp1 == X_k + (dt/6.0) * (f_k + 4*f_mid + f_kp1))
    
    # Control bounds
    opti.subject_to(opti.bounded(SIGMA_MIN, sigma, SIGMA_MAX))
    
    # State bounds
    opti.subject_to(r >= mars.radius)
    opti.subject_to(V >= 10.0)
    opti.subject_to(opti.bounded(deg(-89), gam, deg(89)))
    
    # Control rate constraints
    for k in range(N_SEGMENTS - 1):
        sigma_dot = (sigma[k+1] - sigma[k]) / dt
        opti.subject_to(opti.bounded(SIGMA_DOT_MIN, sigma_dot, SIGMA_DOT_MAX))
    
    # Control acceleration constraints
    for k in range(N_SEGMENTS - 2):
        sigma_dot_k = (sigma[k+1] - sigma[k]) / dt
        sigma_dot_kp1 = (sigma[k+2] - sigma[k+1]) / dt
        sigma_ddot = (sigma_dot_kp1 - sigma_dot_k) / dt
        opti.subject_to(opti.bounded(SIGMA_DOT_DOT_MIN, sigma_ddot, SIGMA_DOT_DOT_MAX))
    
    # Time step bounds
    opti.subject_to(opti.bounded(0.1, dt, 5.0))
    
    # Terminal constraints (deployment envelope)
    # Terminal constraints (deployment envelope)
    # Terminal constraints (deployment envelope) – unified with deploy_region
    MACH_MIN = deploy_region.mach_min
    MACH_MAX = deploy_region.mach_max
    Q_MIN    = deploy_region.q_min
    Q_MAX    = deploy_region.q_max
    H_MIN    = deploy_region.h_min

    
    r_f = r[-1]
    V_f = V[-1]
    h_f = r_f - mars.radius
    
    rho_f = mars.rho0 * ca.exp(-h_f / mars.Hs)
    q_f = 0.5 * rho_f * V_f**2
    
    h_km_f = h_f / 1e3
    T_f = 1.4e-13 * h_km_f**3 - 8.85e-9 * h_km_f**2 - 1.245e-3 * h_km_f + 205.36
    a_sound_f = ca.sqrt(mars.gamma_gas * mars.R_gas * ca.fmax(T_f, 1.0))
    mach_f = V_f / ca.fmax(a_sound_f, 1e-6)
    
    opti.subject_to(opti.bounded(MACH_MIN, mach_f, MACH_MAX))
    opti.subject_to(opti.bounded(Q_MIN, q_f, Q_MAX))
    opti.subject_to(h_f >= H_MIN)
    
    # Initial guess from reachable set
    print("Generating initial guess from reachable set parameters...")
    
    sigma1_rad = np.deg2rad(best_target['sigma1'])
    sigma2_rad = np.deg2rad(best_target['sigma2'])
    
    # Use trajectory duration from reachable set (no need to re-estimate)
    t_actual = best_target['t_final']
    print(f"Using trajectory duration from reachable set: {t_actual:.1f} s")
    
    # Generate initial guess with properly normalized bank profile
    state_sim = state0.copy()
    X_guess = np.zeros((6, N_SEGMENTS + 1))
    X_guess[:, 0] = state0
    
    def eval_terminal_from_guess(X_guess):
        r_f = X_guess[0, -1]
        lon_f = X_guess[1, -1]
        lat_f = X_guess[2, -1]
        V_f = X_guess[3, -1]
        gam_f = X_guess[4, -1]
        psi_f = X_guess[5, -1]

        h_f = r_f - mars.radius
        rho_f = mars.rho0 * np.exp(-h_f / mars.Hs)
        q_f = 0.5 * rho_f * V_f**2

        h_km_f = h_f / 1e3
        T_f = 1.4e-13 * h_km_f**3 - 8.85e-9 * h_km_f**2 - 1.245e-3 * h_km_f + 205.36
        a_f = np.sqrt(mars.gamma_gas * mars.R_gas * max(T_f, 1.0))
        mach_f = V_f / max(a_f, 1e-6)

        # miss distance using same formula as in the Opti problem
        dlat = lat_f - best_target['lat']
        dlon = lon_f - best_target['lon']
        a = np.sin(dlat/2)**2 + np.cos(lat_f)*np.cos(best_target['lat'])*np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        miss_distance = mars.radius * c   # [m]
        print("---- INITIAL GUESS TERMINAL STATE ----")
        print(f"h_f        = {h_f/1e3:.2f} km (limit >= {deploy_region.h_min/1e3:.2f} km)")
        print(f"Mach_f     = {mach_f:.3f} (limits [{deploy_region.mach_min}, {deploy_region.mach_max}])")
        print(f"q_f        = {q_f:.1f} Pa (limits [{deploy_region.q_min}, {deploy_region.q_max}])")
        print(f"miss_dist  = {miss_distance/1e3:.2f} km (limit <= 5.00 km)")
        print(f"gamma_f    = {np.rad2deg(gam_f):.2f} deg")
    
    # Use actual trajectory duration to determine dt
    dt_guess = (t_actual+0.25) / N_SEGMENTS
    print(f"Initial dt guess: {dt_guess:.3f} s/segment (total time: {t_actual:.1f} s)")
    
    # Now the bank profile is properly timed
    deployed = False
    for k in range(N_SEGMENTS):
        t_k = k * dt_guess
        sigma_k = bank_profile_three_section(t_k, t_actual, sigma1_rad, sigma2_rad)
        print(f"Segment {k+1}/{N_SEGMENTS}, t={t_k:.1f}s, sigma={np.rad2deg(sigma_k):.2f} deg")
        state_sim = rk4_step(eom_numpy, state_sim, sigma_k, dt_guess)
        X_guess[:, k+1] = state_sim
        
        # Check deployment trigger
        h_sim, _, q_sim, _, _ = compute_loads(state_sim)
        V_sim = state_sim[3]
        if deploy_trigger(h_sim, V_sim, q_sim):
            deployed = True
            # Fill remaining segments with final state
            for j in range(k+2, N_SEGMENTS+1):
                X_guess[:, j] = state_sim
            print(f"Deployment conditions met at segment {k+1}/{N_SEGMENTS} (t={t_k:.1f}s)")
            break
    
    if not deployed:
        print(f"WARNING: Deployment conditions not met within {N_SEGMENTS} segments")
    
    print(f"Initial guess: h_final = {(X_guess[0,-1] - mars.radius)/1e3:.1f} km")
    print(f"Initial guess: gamma_final = {np.rad2deg(X_guess[4,-1]):.1f} deg")
    
    opti.set_initial(X, X_guess)
    eval_terminal_from_guess(X_guess)
    
    # Control initial guess with properly normalized bank profile
    U_guess = np.zeros((1, N_SEGMENTS))
    for k in range(N_SEGMENTS):
        t_k = k * dt_guess
        U_guess[0, k] = bank_profile_three_section(t_k, t_actual, sigma1_rad, sigma2_rad)
    
    opti.set_initial(U, U_guess)
    # After computing dt_guess:
    opti.set_initial(dt, dt_guess)

    # Replace your old dt bounds with tighter ones:
    opti.subject_to(opti.bounded(0.5 * dt_guess, dt, 1.5 * dt_guess))

    # Solver options
    opts = {
        'ipopt.print_level': 5,
        'ipopt.max_iter': 3000,
        'ipopt.tol': 1e-6,
        'ipopt.acceptable_tol': 1e-4,
        'ipopt.mu_strategy': 'adaptive',
        'ipopt.nlp_scaling_method': 'gradient-based',
        'print_time': True
    }
    
    opti.solver('ipopt', opts)
    
    print("\nStarting IPOPT solver...\n")
    start_time = time.perf_counter()
    
    try:
        sol = opti.solve()
        success = True
        print("\n" + "="*70)
        print("OPTIMIZATION SUCCESSFUL")
        print("="*70)
    except RuntimeError as e:
        print("\n" + "="*70)
        print("OPTIMIZATION FAILED")
        print("="*70)
        print(f"Error: {e}")
        sol = opti.debug
        success = False
    
    elapsed = time.perf_counter() - start_time
    
    # Extract solution
    X_sol = sol.value(X)
    U_sol = sol.value(U)
    dt_sol = sol.value(dt)
    t_sol = np.arange(N_SEGMENTS + 1) * dt_sol
    
    if U_sol.ndim == 2:
        U_sol_flat = U_sol[0, :]
    else:
        U_sol_flat = U_sol
    
    # Build history
    hist = {
        "t": t_sol,
        "h": (X_sol[0, :] - mars.radius) / 1e3,
        "V": X_sol[3, :],
        "gam": X_sol[4, :],
        "psi": X_sol[5, :],
        "lat": X_sol[2, :],
        "lon": X_sol[1, :],
        "sigma": np.append(U_sol_flat, U_sol_flat[-1]),
    }
    
    # Compute derived quantities
    hist["rho"] = []
    hist["q"] = []
    hist["mach"] = []
    hist["A"] = []
    hist["Qdot"] = []
    
    for i in range(N_SEGMENTS + 1):
        r_i = X_sol[0, i]
        V_i = X_sol[3, i]
        h_i = r_i - mars.radius
        
        rho_i = mars.rho0 * np.exp(-h_i / mars.Hs)
        q_i = 0.5 * rho_i * V_i**2
        
        D_i = 0.5 * rho_i * V_i**2 / veh.beta
        L_i = veh.L_over_D * D_i
        A_i = np.sqrt(D_i**2 + L_i**2) / 9.80665
        
        Qdot_i = veh.k_heat_flux * (rho_i**veh.N) * (V_i**veh.M)
        
        h_km_i = h_i / 1e3
        T_i = 1.4e-13 * h_km_i**3 - 8.85e-9 * h_km_i**2 - 1.245e-3 * h_km_i + 205.36
        a_i = np.sqrt(mars.gamma_gas * mars.R_gas * max(T_i, 1.0))
        mach_i = V_i / max(a_i, 1e-6)
        
        hist["rho"].append(rho_i)
        hist["q"].append(q_i)
        hist["mach"].append(mach_i)
        hist["A"].append(A_i)
        hist["Qdot"].append(Qdot_i)
    
    for key in ["rho", "q", "mach", "A", "Qdot"]:
        hist[key] = np.array(hist[key])
    
    print(f"\nSolution time: {elapsed:.1f} s")
    print(f"Final altitude: {hist['h'][-1]:.2f} km")
    print(f"Final lat: {np.rad2deg(hist['lat'][-1]):.3f}°, lon: {np.rad2deg(hist['lon'][-1]):.3f}°")
    print(f"Final Mach: {hist['mach'][-1]:.2f}")
    print(f"Final q: {hist['q'][-1]:.1f} Pa")
    print(f"Miss distance: {sol.value(miss_distance)/1e3:.2f} km")
    print(f"Trajectory duration: {t_sol[-1]:.1f} s")
    print(f"Time step: {dt_sol:.3f} s")
    
    return hist, success

# ============================================================
# COMPREHENSIVE PLOTTING
# ============================================================

def plot_comprehensive_results(best_target, all_samples, RD_all, RC_all, hist_opt):
    """Create comprehensive plots of reachable set and optimized trajectory."""
    
    # Extract reachable set data
    lat_all = all_samples[:, 0]
    lon_all = all_samples[:, 1]
    ALT_all = all_samples[:, 2]
    q_all = all_samples[:, 3]
    A_all = all_samples[:, 4]
    Qdot_all = all_samples[:, 5]
    
    # ===== FIGURE 1: REACHABLE SET FOOTPRINTS =====
    fig1 = plt.figure(figsize=(16, 12))
    
    # Lat/Lon footprint
    ax1 = plt.subplot(2, 2, 1)
    sc1 = ax1.scatter(np.rad2deg(lon_all), np.rad2deg(lat_all), 
                      c=ALT_all, cmap='viridis', s=50, alpha=0.6)
    ax1.plot(np.rad2deg(best_target['lon']), np.rad2deg(best_target['lat']), 
             'r*', ms=25, label='Best Target', markeredgecolor='darkred', markeredgewidth=2)
    ax1.set_xlabel('Longitude [deg]', fontsize=11)
    ax1.set_ylabel('Latitude [deg]', fontsize=11)
    ax1.set_title('Reachable Set: Lat/Lon Footprint', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    plt.colorbar(sc1, ax=ax1, label='Altitude [km]')
    
    # Downrange/Crossrange footprint
    ax2 = plt.subplot(2, 2, 2)
    sc2 = ax2.scatter(RD_all, RC_all, c=ALT_all, cmap='viridis', s=50, alpha=0.6)
    ax2.plot(best_target['downrange'], best_target['crossrange'], 
             'r*', ms=25, label='Best Target', markeredgecolor='darkred', markeredgewidth=2)
    ax2.set_xlabel('Downrange [km]', fontsize=11)
    ax2.set_ylabel('Crossrange [km]', fontsize=11)
    ax2.set_title('Reachable Set: Downrange/Crossrange', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    plt.colorbar(sc2, ax=ax2, label='Altitude [km]')
    
    # Max acceleration footprint
    ax3 = plt.subplot(2, 2, 3)
    sc3 = ax3.scatter(RD_all, RC_all, c=A_all, cmap='hot', s=50, alpha=0.6, vmax=constraints.A_max)
    ax3.plot(best_target['downrange'], best_target['crossrange'], 
             'b*', ms=25, label='Best Target', markeredgecolor='darkblue', markeredgewidth=2)
    ax3.set_xlabel('Downrange [km]', fontsize=11)
    ax3.set_ylabel('Crossrange [km]', fontsize=11)
    ax3.set_title('Max Acceleration [g]', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3)
    plt.colorbar(sc3, ax=ax3, label='Max A [g]')
    
    # Max dynamic pressure footprint
    ax4 = plt.subplot(2, 2, 4)
    sc4 = ax4.scatter(RD_all, RC_all, c=q_all/1e3, cmap='plasma', s=50, alpha=0.6, vmax=constraints.q_max/1e3)
    ax4.plot(best_target['downrange'], best_target['crossrange'], 
             'g*', ms=25, label='Best Target', markeredgecolor='darkgreen', markeredgewidth=2)
    ax4.set_xlabel('Downrange [km]', fontsize=11)
    ax4.set_ylabel('Crossrange [km]', fontsize=11)
    ax4.set_title('Max Dynamic Pressure [kPa]', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=10)
    ax4.grid(True, alpha=0.3)
    plt.colorbar(sc4, ax=ax4, label='Max q [kPa]')
    
    plt.tight_layout()
    
    # ===== FIGURE 2: OPTIMIZED TRAJECTORY TIME HISTORIES =====
    fig2 = plt.figure(figsize=(16, 12))
    
    t = hist_opt["t"]
    h = hist_opt["h"]
    V = hist_opt["V"]
    gam = np.rad2deg(hist_opt["gam"])
    psi = np.rad2deg(hist_opt["psi"])
    sigma = np.rad2deg(hist_opt["sigma"])
    q = hist_opt["q"]
    mach = hist_opt["mach"]
    A = hist_opt["A"]
    Qdot = hist_opt["Qdot"]
    
    # Calculate control rates
    sigma_rad = hist_opt["sigma"]
    sigma_dot = np.gradient(sigma_rad, t)
    sigma_ddot = np.gradient(sigma_dot, t)
    
    axes = []
    for i in range(9):
        axes.append(plt.subplot(3, 3, i+1))
    
    # Altitude
    axes[0].plot(t, h, 'b-', lw=2)
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Altitude [km]')
    axes[0].set_title('Altitude')
    axes[0].grid(True, alpha=0.3)
    
    # Velocity
    axes[1].plot(t, V, 'r-', lw=2)
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Velocity [m/s]')
    axes[1].set_title('Velocity')
    axes[1].grid(True, alpha=0.3)
    
    # Dynamic pressure
    axes[2].plot(t, q/1e3, 'g-', lw=2)
    axes[2].axhline(constraints.q_max/1e3, ls='--', color='red', alpha=0.5, label='Limit')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('q [kPa]')
    axes[2].set_title('Dynamic Pressure')
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)
    
    # Mach number
    axes[3].plot(t, mach, 'purple', lw=2)
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('Mach [-]')
    axes[3].set_title('Mach Number')
    axes[3].grid(True, alpha=0.3)
    
    # Flight path angle
    axes[4].plot(t, gam, 'orange', lw=2)
    axes[4].set_xlabel('Time [s]')
    axes[4].set_ylabel('γ [deg]')
    axes[4].set_title('Flight Path Angle')
    axes[4].grid(True, alpha=0.3)
    
    # Heading angle
    axes[5].plot(t, psi, 'brown', lw=2)
    axes[5].set_xlabel('Time [s]')
    axes[5].set_ylabel('ψ [deg]')
    axes[5].set_title('Heading Angle')
    axes[5].grid(True, alpha=0.3)
    
    # Bank angle
    axes[6].plot(t, sigma, 'k-', lw=2)
    axes[6].axhline(81, ls='--', color='red', alpha=0.5, label='Limit')
    axes[6].axhline(-81, ls='--', color='red', alpha=0.5)
    axes[6].set_xlabel('Time [s]')
    axes[6].set_ylabel('σ [deg]')
    axes[6].set_title('Bank Angle')
    axes[6].legend(fontsize=8)
    axes[6].grid(True, alpha=0.3)
    
    # Bank rate
    axes[7].plot(t, np.rad2deg(sigma_dot), 'b-', lw=2)
    axes[7].axhline(20, ls='--', color='red', alpha=0.5, label='Limit')
    axes[7].axhline(-20, ls='--', color='red', alpha=0.5)
    axes[7].set_xlabel('Time [s]')
    axes[7].set_ylabel('σ̇ [deg/s]')
    axes[7].set_title('Bank Rate')
    axes[7].legend(fontsize=8)
    axes[7].grid(True, alpha=0.3)
    
    # Bank acceleration
    axes[8].plot(t, np.rad2deg(sigma_ddot), 'r-', lw=2)
    axes[8].axhline(5, ls='--', color='red', alpha=0.5, label='Limit')
    axes[8].axhline(-5, ls='--', color='red', alpha=0.5)
    axes[8].set_xlabel('Time [s]')
    axes[8].set_ylabel('σ̈ [deg/s²]')
    axes[8].set_title('Bank Acceleration')
    axes[8].legend(fontsize=8)
    axes[8].grid(True, alpha=0.3)
    
    plt.suptitle('Optimized Trajectory: State and Control Histories', fontsize=14, fontweight='bold', y=0.995)
    plt.tight_layout()
    
    # ===== FIGURE 3: PATH CONSTRAINTS =====
    fig3, axes3 = plt.subplots(2, 2, figsize=(14, 10))
    axes3 = axes3.flatten()
    
    # Acceleration
    axes3[0].plot(t, A, 'r-', lw=2.5, label='Acceleration')
    axes3[0].axhline(constraints.A_max, ls='--', color='red', alpha=0.5, lw=2, label='Limit')
    axes3[0].set_xlabel('Time [s]', fontsize=11)
    axes3[0].set_ylabel('Acceleration [g]', fontsize=11)
    axes3[0].set_title('Total Acceleration', fontsize=12, fontweight='bold')
    axes3[0].legend(fontsize=10)
    axes3[0].grid(True, alpha=0.3)
    
    # Dynamic pressure
    axes3[1].plot(t, q/1e3, 'g-', lw=2.5, label='Dynamic Pressure')
    axes3[1].axhline(constraints.q_max/1e3, ls='--', color='red', alpha=0.5, lw=2, label='Limit')
    axes3[1].set_xlabel('Time [s]', fontsize=11)
    axes3[1].set_ylabel('q [kPa]', fontsize=11)
    axes3[1].set_title('Dynamic Pressure', fontsize=12, fontweight='bold')
    axes3[1].legend(fontsize=10)
    axes3[1].grid(True, alpha=0.3)
    
    # Heat rate
    axes3[2].plot(t, Qdot/1e3, 'orange', lw=2.5, label='Heat Rate')
    axes3[2].axhline(constraints.Qdot_max/1e3, ls='--', color='red', alpha=0.5, lw=2, label='Limit')
    axes3[2].set_xlabel('Time [s]', fontsize=11)
    axes3[2].set_ylabel('Heat Rate [kW/m²]', fontsize=11)
    axes3[2].set_title('Heat Rate', fontsize=12, fontweight='bold')
    axes3[2].legend(fontsize=10)
    axes3[2].grid(True, alpha=0.3)
    
     # Altitude-Velocity envelope (consistent with deploy_region)
    V_range = np.linspace(300.0, 500.0, 300)   # [m/s]
    h_grid  = np.linspace(deploy_region.h_min, 20e3, 400)  # [m]

    h_lower = np.full_like(V_range, np.nan, dtype=float)
    h_upper = np.full_like(V_range, np.nan, dtype=float)

    for i, V_i in enumerate(V_range):
        # Atmospheric quantities on the altitude grid
        rho = mars.rho0 * np.exp(-h_grid / mars.Hs)
        q   = 0.5 * rho * V_i**2

        # Mach number using the same sound speed model
        a = np.array([mars.sound_speed(h) for h in h_grid])
        mach = V_i / np.maximum(a, 1e-6)

        mask = (
            (deploy_region.mach_min <= mach) &
            (mach <= deploy_region.mach_max) &
            (deploy_region.q_min    <= q) &
            (q <= deploy_region.q_max)
        )

        if np.any(mask):
            h_valid_km = h_grid[mask] / 1e3
            h_lower[i] = h_valid_km.min()
            h_upper[i] = h_valid_km.max()

    valid = ~np.isnan(h_lower) & ~np.isnan(h_upper)

    axes3[3].fill_between(
        V_range[valid], h_lower[valid], h_upper[valid],
        alpha=0.3, color='green', label='Safe Envelope'
    )
    axes3[3].plot(V_range[valid], h_lower[valid], 'g-', lw=2)
    axes3[3].plot(V_range[valid], h_upper[valid], 'g-', lw=2)

    # Plot trajectory and final deployment point
    axes3[3].plot(V, h, 'k-', lw=2.5, label='Trajectory', zorder=5)
    axes3[3].plot(V[-1], h[-1], 'r*', ms=20, label='Deployment', zorder=10,
                  markeredgecolor='darkred', markeredgewidth=1.5)

    axes3[3].set_xlabel('Velocity [m/s]', fontsize=11)
    axes3[3].set_ylabel('Altitude [km]', fontsize=11)
    axes3[3].set_title('Altitude-Velocity Envelope', fontsize=12, fontweight='bold')
    axes3[3].legend(fontsize=10)
    axes3[3].grid(True, alpha=0.3)

    
    plt.suptitle('Path Constraints Verification', fontsize=14, fontweight='bold', y=0.995)
    plt.tight_layout()
    
    # ===== FIGURE 4: GROUND TRACK =====
    fig4, ax4 = plt.subplots(figsize=(12, 10))
    
    lat_traj = np.rad2deg(hist_opt["lat"])
    lon_traj = np.rad2deg(hist_opt["lon"])
    
    # Plot reachable set
    ax4.scatter(np.rad2deg(lon_all), np.rad2deg(lat_all), 
                c='lightgray', s=30, alpha=0.4, label='Reachable Set')
    
    # Plot optimized trajectory
    sc4 = ax4.scatter(lon_traj, lat_traj, c=t, cmap='viridis', s=50, 
                      label='Optimized Trajectory', zorder=3, edgecolors='k', linewidths=0.5)
    
    # Mark important points
    ax4.plot(np.rad2deg(lon0), np.rad2deg(lat0), 'go', ms=15, 
             label='Entry', zorder=5, markeredgecolor='darkgreen', markeredgewidth=2)
    ax4.plot(lon_traj[-1], lat_traj[-1], 'bs', ms=15, 
             label='Deployment', zorder=5, markeredgecolor='darkblue', markeredgewidth=2)
    ax4.plot(np.rad2deg(best_target['lon']), np.rad2deg(best_target['lat']), 
             'r*', ms=25, label='Target', zorder=5, markeredgecolor='darkred', markeredgewidth=2)
    
    ax4.set_xlabel('Longitude [deg]', fontsize=12)
    ax4.set_ylabel('Latitude [deg]', fontsize=12)
    ax4.set_title('Ground Track: Reachable Set + Optimized Trajectory', fontsize=13, fontweight='bold')
    ax4.legend(loc='best', fontsize=11)
    ax4.grid(True, alpha=0.3)
    plt.colorbar(sc4, ax=ax4, label='Time [s]')
    plt.tight_layout()
    
    plt.show()

# ============================================================
# MAIN EXECUTION
# ============================================================

def main():
    """Main execution: reachable set → target selection → collocation optimization."""
    
    print("\n" + "="*80)
    print("INTEGRATED TRAJECTORY OPTIMIZATION")
    print("="*80)
    print("Step 1: Build reachable set via control parameter sweep")
    print("Step 2: Select best target using cost function")
    print("Step 3: Optimize trajectory to target using collocation")
    print("="*80)
    
    # Step 1: Build reachable set
    sigma1_grid = np.linspace(np.deg2rad(-81), np.deg2rad(81), 10)
    sigma2_grid = np.linspace(np.deg2rad(-81), np.deg2rad(81), 10)
    
    all_samples = build_reachable_set(lat0, lon0, sigma1_grid, sigma2_grid, dt=0.25, tmax=4000.0)
    
    # Step 2: Find best target
    result = find_best_target(all_samples, lat0, lon0)
    
    if result is None:
        print("ERROR: Cannot proceed without valid target!")
        return
    
    best_target, all_samples, RD_all, RC_all = result
    
    # Step 3: Optimize trajectory to target
    hist_opt, success = solve_collocation_with_target(best_target)
    
    if not success:
        print("\nWARNING: Optimization did not converge to optimal solution")
        print("Plotting debug solution...")
    
    # Step 4: Plot comprehensive results
    print("\n" + "="*80)
    print("GENERATING COMPREHENSIVE PLOTS")
    print("="*80)
    
    plot_comprehensive_results(best_target, all_samples, RD_all, RC_all, hist_opt)
    
    print("\n" + "="*80)
    print("INTEGRATION COMPLETE")
    print("="*80)

if __name__ == "__main__":
    main()
