"""
Mars Entry Integrated Optimization (Clean Version)
==================================================

Pipeline:
1. Build reachable set over (σ1, σ2) grid using a two-pass simulation:
   - First pass: approximate deploy time
   - Second pass: re-simulate with bank profile normalized to deploy time

2. Select "best" target:
   - Path constraints satisfied (q, A, Qdot)
   - Optional crossrange restriction
   - Maximum deploy altitude

3. Optimize trajectory using direct collocation (CasADi):
   - Terminal position within tolerance of best target
   - Final state in deploy envelope
   - Path constraints on q, A, Qdot
   - Bank angle / rate / acceleration constraints
   - Objective: maximize final altitude and shape γ and deploy conditions

4. Plot:
   - Reachable set footprints (lat/lon + metrics)
   - Candidate trajectory (from reachable set)
   - Optimized trajectory time histories
   - Ground track & diagnostic plots
"""

import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from dataclasses import dataclass
from spherical_metrics import spherical_metrics

# ============================================================
# 1️⃣ MODELS & CONSTANTS
# ============================================================

@dataclass
class Mars:
    radius: float = 3396.2e3
    mu: float = 4.282837e13
    rho0: float = 0.02
    Hs: float = 11.1e3
    omega: float = 7.088e-5
    gamma_gas: float = 1.3
    R_gas: float = 191.0

    def density(self, h):
        return self.rho0 * np.exp(-h / self.Hs)

    def sound_speed(self, h):
        """Simple polynomial-based temperature → sound speed model."""
        h_km = h / 1e3
        T = 1.4e-13*h_km**3 - 8.85e-9*h_km**2 - 1.245e-3*h_km + 205.36
        return np.sqrt(self.gamma_gas * self.R_gas * max(T, 1.0))

@dataclass
class Vehicle:
    beta: float = 135.0          # ballistic coefficient
    L_over_D: float = 0.24
    k_heat_flux: float = 5.3697e-5
    N: float = 0.5
    M: float = 3.0

@dataclass
class Constraints:
    q_max: float = 13000.0       # Pa
    A_max: float = 15.0          # g
    Qdot_max: float = 5e5        # W/m^2
    sigma_max: float = 81.0      # deg
    sigma_dot_max: float = 20.0  # deg/s
    sigma_ddot_max: float = 5.0  # deg/s^2

@dataclass
class DeployRegion:
    mach_min: float = 1.4
    mach_max: float = 2.2
    q_min: float = 300.0         # Pa
    q_max: float = 800.0         # Pa
    h_min: float = 6e3           # m


mars = Mars()
veh = Vehicle()
cons = Constraints()
deploy = DeployRegion()

# Entry state
deg = np.deg2rad
h0 = 125e3
r0 = mars.radius + h0
V0 = 5000.0
gam0 = deg(-12.0)
psi0 = deg(-2.8758)
lat0 = deg(-21.3)
lon0 = deg(-176.40167)

state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])

# ============================================================
# 2️⃣ BASIC UTILITIES
# ============================================================

def great_circle_distance(lat1, lon1, lat2, lon2, R):
    """Great-circle distance [m] between points (radians)."""
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1)*np.cos(lat2)*np.sin(dlon/2)**2
    c = 2*np.arctan2(np.sqrt(a), np.sqrt(1-a))
    return R*c

# ============================================================
# 3️⃣ DYNAMICS & LOADS (NumPy)
# ============================================================

def eom_numpy(state, sigma):
    """3DOF equations of motion including planetary rotation."""
    r, lon, lat, V, gam, psi = state
    g = mars.mu / r**2
    h = r - mars.radius
    rho = mars.density(h)

    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D
    Vh = V * np.cos(gam)

    cos_lat = max(np.cos(lat), 1e-6)
    V_safe = max(V, 1e-6)
    cos_gam_safe = max(np.cos(gam), 1e-6)

    r_dot  = V * np.sin(gam)
    lon_dot = (Vh * np.sin(psi)) / (r * cos_lat)
    lat_dot = (Vh * np.cos(psi)) / r

    V_dot = -D - g * np.sin(gam) + mars.omega**2 * r * np.cos(lat) * \
        (np.sin(gam)*np.cos(lat) - np.cos(gam)*np.sin(lat)*np.cos(psi))

    gam_dot = (L*np.cos(sigma)/V_safe) + (V/r - g/V_safe)*np.cos(gam) + \
              2*mars.omega*np.cos(lat)*np.sin(psi) + \
              (mars.omega**2*r/V_safe)*np.cos(lat)* \
              (np.cos(gam)*np.cos(lat) + np.sin(gam)*np.sin(lat)*np.cos(psi))

    psi_dot = (L*np.sin(sigma))/(V_safe*cos_gam_safe) + \
              (Vh/r)*np.sin(psi)*np.tan(lat) - \
              2*mars.omega*(np.tan(gam)*np.cos(lat)*np.cos(psi) - np.sin(lat)) + \
              (mars.omega**2*r/(V_safe*cos_gam_safe))*np.sin(lat)*np.cos(lat)*np.sin(psi)

    return np.array([r_dot, lon_dot, lat_dot, V_dot, gam_dot, psi_dot])

def rk4_step(f, state, sigma, dt):
    k1 = f(state, sigma)
    k2 = f(state + 0.5*dt*k1, sigma)
    k3 = f(state + 0.5*dt*k2, sigma)
    k4 = f(state + dt*k3, sigma)
    return state + dt/6*(k1 + 2*k2 + 2*k3 + k4)

def aero_loads(state):
    r, _, _, V, _, _ = state
    h = r - mars.radius
    rho = mars.density(h)
    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D
    q = 0.5 * rho * V**2
    A = np.sqrt(D**2 + L**2) / 9.80665
    Qdot = veh.k_heat_flux * (rho**veh.N) * (V**veh.M)
    return h, q, A, Qdot

def deploy_trigger(h, V, q):
    a = mars.sound_speed(h)
    mach = V / max(a, 1e-6)
    return (
        (deploy.mach_min <= mach <= deploy.mach_max) and
        (deploy.q_min <= q <= deploy.q_max) and
        (h >= deploy.h_min)
    )

def bank_profile_three_section(t, T, sigma1, sigma2):
    """
    Three-section bank profile in time-normalized coordinates:
      0 ≤ τ < 0.35: σ = σ1
      0.35 ≤ τ < 0.50: linear blend from σ1 → σ2
      0.50 ≤ τ ≤ 1   : σ = σ2
    """
    if T <= 0:
        return sigma1
    tau = t / T
    if tau < 0.35:
        return sigma1
    elif tau < 0.50:
        alpha = (tau - 0.35) / (0.15 + 1e-9)  # 0 at τ=0.35, 1 at τ=0.50
        return sigma1 * (1 - alpha) + sigma2 * alpha  # linear interpolation
    else:
        return sigma2

# ============================================================
# 4️⃣ TWO-PASS SIMULATION (CONSISTENT BANK TIMING)
# ============================================================

def simulate_entry_two_pass(sigma1, sigma2, dt=0.25, tmax=4000.0):
    """
    Two-pass simulation to ensure bank profile is normalized to deploy time.

    Pass 1:
      - Use a provisional duration T_guess = tmax
      - Integrate with bank_profile_three_section(t, T_guess, σ1, σ2)
      - Find deploy time t_dep (when deploy_trigger is first met)
      - If no deploy or path constraint violation → invalid

    Pass 2:
      - Reintegrate from state0 with bank_profile_three_section(t, t_dep, σ1, σ2),
        so the same 3-section shape is normalized to actual deploy time
      - Compute final state, deploy loads, and max loads

    Returns dict or None if invalid:
      {
        lat, lon, h, V, gam, psi,
        max_q, max_A, max_Qdot,
        q_dep, A_dep, Qdot_dep, mach_dep,
        sigma1, sigma2, t_dep
      }
    """
    # ----- Pass 1: estimate deploy time -----
    t = 0.0
    state = state0.copy()

    max_q = max_A = max_Qdot = 0.0
    t_dep = None

    while t < tmax:
        sigma = bank_profile_three_section(t, tmax, sigma1, sigma2)
        state = rk4_step(eom_numpy, state, sigma, dt)
        h, q, A, Qdot = aero_loads(state)
        max_q = max(max_q, q)
        max_A = max(max_A, A)
        max_Qdot = max(max_Qdot, Qdot)

        if q > cons.q_max or A > cons.A_max or Qdot > cons.Qdot_max:
            return None

        if deploy_trigger(h, state[3], q) or h <= 0:
            t_dep = t
            break

        t += dt

    if t_dep is None:
        return None  # never deployed

    # ----- Pass 2: re-simulate with normalized profile -----
    T = max(t_dep, dt)
    t = 0.0
    state = state0.copy()
    max_q = max_A = max_Qdot = 0.0  # recompute with normalized timing

    while t < tmax:
        sigma = bank_profile_three_section(t, T, sigma1, sigma2)
        state = rk4_step(eom_numpy, state, sigma, dt)
        h, q, A, Qdot = aero_loads(state)

        max_q = max(max_q, q)
        max_A = max(max_A, A)
        max_Qdot = max(max_Qdot, Qdot)

        if q > cons.q_max or A > cons.A_max or Qdot > cons.Qdot_max:
            return None

        if deploy_trigger(h, state[3], q) or h <= 0:
            break

        t += dt

    # Final state at deploy
    r, lon, lat, V, gam, psi = state
    h_final, q_final, A_final, Qdot_final = aero_loads(state)
    a_final = mars.sound_speed(h_final)
    mach_final = V / max(a_final, 1e-6)

    return dict(
        lat   = lat,
        lon   = lon,
        h     = r - mars.radius,
        V     = V,
        gam   = gam,
        psi   = psi,
        max_q   = max_q,
        max_A   = max_A,
        max_Qdot = max_Qdot,
        q_dep    = q_final,
        A_dep    = A_final,
        Qdot_dep = Qdot_final,
        mach_dep = mach_final,
        sigma1 = np.rad2deg(sigma1),
        sigma2 = np.rad2deg(sigma2),
        t_dep  = t
    )

# ============================================================
# 5️⃣ REACHABLE SET & TARGET SELECTION
# ============================================================

def build_reachable_set(n_grid=10, dt=0.25, tmax=4000.0):
    sigma_grid = np.deg2rad(np.linspace(-81, 81, n_grid))
    samples = []
    for s1 in sigma_grid:
        for s2 in sigma_grid:
            res = simulate_entry_two_pass(s1, s2, dt=dt, tmax=tmax)
            if res is not None:
                samples.append(res)
    return samples

def select_best_target(samples, max_crossrange_km=5.0):
    """
    Filter samples:
      - path constraints already enforced
      - optional |crossrange| < max_crossrange_km
    Then pick the one with maximum deploy altitude.
    """
    if not samples:
        return None

    # Use a reference target slightly ahead of entry point for proper coordinate frame
    lat_ref = lat0 + np.deg2rad(1.0)  # 1 degree ahead in latitude
    lon_ref = lon0  # Same longitude

    filtered = []
    for s in samples:
        lat_f = s['lat']
        lon_f = s['lon']
        # Use spherical_metrics to compute true crossrange
        _, _, _, RC, RD, RD_go, Rgo = spherical_metrics(
            lat0, lon0, lat_f, lon_f, lat_ref, lon_ref, mars.radius
        )
        if abs(RC/1e3) <= max_crossrange_km:
            filtered.append(s)

    if not filtered:
        filtered = samples  # if nothing passes crossrange, fallback

    best = max(filtered, key=lambda s: s['h'])

    # Add crossrange & downrange using spherical_metrics BEFORE printing
    lat_f = best['lat']
    lon_f = best['lon']
    lat_ref = lat0 + np.deg2rad(1.0)
    lon_ref = lon0
    _, _, _, RC, RD, RD_go, Rgo = spherical_metrics(
        lat0, lon0, lat_f, lon_f, lat_ref, lon_ref, mars.radius
    )
    best['crossrange_km'] = RC / 1e3
    best['downrange_km'] = RD / 1e3

    print("\nBest target (reachable set):")
    print(f"  h_dep      = {best['h']/1e3:7.2f} km")
    print(f"  lat        = {np.rad2deg(best['lat']):7.3f} deg")
    print(f"  lon        = {np.rad2deg(best['lon']):7.3f} deg")
    print(f"  crossrange = {best['crossrange_km']:7.2f} km")
    print(f"  downrange  = {best['downrange_km']:7.2f} km")
    print(f"  σ1, σ2     = {best['sigma1']:6.1f}, {best['sigma2']:6.1f} deg")
    print(f"  t_dep      = {best['t_dep']:7.2f} s")
    print(f"  max q      = {best['max_q']:8.1f} Pa")
    print(f"  max A      = {best['max_A']:5.2f} g")
    print(f"  max Q̇      = {best['max_Qdot']:8.1f} W/m²")
    print(f"  q_dep      = {best['q_dep']:8.1f} Pa")
    print(f"  Mach_dep   = {best['mach_dep']:7.3f}")

    return best

# ============================================================
# 6️⃣ DYNAMICS (CasADi) FOR COLLOCATION
# ============================================================

def eom_ca(x, sigma):
    """CasADi version of EOM, consistent with eom_numpy."""
    r   = x[0]
    lon = x[1]
    lat = x[2]
    V   = x[3]
    gam = x[4]
    psi = x[5]

    g = mars.mu / (r**2)
    h = r - mars.radius
    rho = mars.rho0 * ca.exp(-h / mars.Hs)

    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D
    Vh = V * ca.cos(gam)

    cos_lat = ca.fmax(ca.cos(lat), 1e-6)
    V_safe = ca.fmax(V, 1e-6)
    cos_gam_safe = ca.fmax(ca.cos(gam), 1e-6)

    rdot = V * ca.sin(gam)
    londot = (Vh*ca.sin(psi)) / (r * cos_lat)
    latdot = (Vh*ca.cos(psi)) / r

    Vdot = -D - g*ca.sin(gam) + mars.omega**2 * r * ca.cos(lat) * \
        (ca.sin(gam)*ca.cos(lat) - ca.cos(gam)*ca.sin(lat)*ca.cos(psi))

    gamdot = (L*ca.cos(sigma)/V_safe) + (V/r - g/V_safe)*ca.cos(gam) + \
             2*mars.omega*ca.cos(lat)*ca.sin(psi) + \
             (mars.omega**2*r/V_safe)*ca.cos(lat)* \
             (ca.cos(gam)*ca.cos(lat) + ca.sin(gam)*ca.sin(lat)*ca.cos(psi))

    psidot = (L*ca.sin(sigma))/(V_safe*cos_gam_safe) + \
             (Vh/r)*ca.sin(psi)*ca.tan(lat) - \
             2*mars.omega*(ca.tan(gam)*ca.cos(lat)*ca.cos(psi) - ca.sin(lat)) + \
             (mars.omega**2*r/(V_safe*cos_gam_safe))*ca.sin(lat)*ca.cos(lat)*ca.sin(psi)

    return ca.vertcat(rdot, londot, latdot, Vdot, gamdot, psidot)

# ============================================================
# 7️⃣ COLLOCATION OPTIMIZATION
# ============================================================

def optimize_to_target(best, N=180):
    """
    Direct collocation with trapezoidal integration.
    Terminal constraints:
      - position within tolerance of best target
      - final state in deploy envelope
    Path constraints on q, A, Qdot.
    Control constraints on σ, σ̇, σ̈.
    """
    opti = ca.Opti()

    X = opti.variable(6, N+1)   # states
    U = opti.variable(1, N)     # bank angle
    dt = opti.variable()        # time step

    # Initial condition
    opti.subject_to(X[:,0] == state0)

    # Dynamics constraints (trapezoidal)
    for k in range(N):
        xk  = X[:,k]
        xk1 = X[:,k+1]
        uk  = U[:,k]

        f1 = eom_ca(xk,  uk)
        f2 = eom_ca(xk1, uk)
        opti.subject_to(xk1 == xk + 0.5*dt*(f1 + f2))

    # Terminal quantities
    r_f = X[0,-1]
    lon_f = X[1,-1]
    lat_f = X[2,-1]
    V_f = X[3,-1]
    gam_f = X[4,-1]

    h_f = r_f - mars.radius
    rho_f = mars.rho0 * ca.exp(-h_f / mars.Hs)
    q_f = 0.5 * rho_f * V_f**2

    h_km_f = h_f / 1e3
    T_f = 1.4e-13*h_km_f**3 - 8.85e-9*h_km_f**2 - 1.245e-3*h_km_f + 205.36
    T_f = ca.fmax(T_f, 1.0)
    a_f = ca.sqrt(mars.gamma_gas * mars.R_gas * T_f)
    mach_f = V_f / a_f

    # Deploy constraints at final point
    opti.subject_to(mach_f >= deploy.mach_min)
    opti.subject_to(mach_f <= deploy.mach_max)
    opti.subject_to(q_f    >= deploy.q_min)
    opti.subject_to(q_f    <= deploy.q_max)
    opti.subject_to(h_f    >= deploy.h_min)

    # Terminal position miss (inequality)
    lat_tgt = best['lat']
    lon_tgt = best['lon']
    dlat = lat_f - lat_tgt
    dlon = lon_f - lon_tgt
    miss = mars.radius * ca.sqrt(dlat**2 + dlon**2)
    opti.subject_to(miss <= 1000.0)  # 1 km tolerance

    # Path constraints at all nodes
    for k in range(N+1):
        xk = X[:,k]
        r_k = xk[0]
        V_k = xk[3]
        h_k = r_k - mars.radius
        rho_k = mars.rho0 * ca.exp(-h_k / mars.Hs)
        q_k = 0.5 * rho_k * V_k**2
        D_k = 0.5 * rho_k * V_k**2 / veh.beta
        L_k = veh.L_over_D * D_k
        A_k = ca.sqrt(D_k**2 + L_k**2) / 9.80665
        Qdot_k = veh.k_heat_flux * (rho_k**veh.N) * (V_k**veh.M)

        opti.subject_to(q_k    <= cons.q_max)
        opti.subject_to(A_k    <= cons.A_max)
        opti.subject_to(Qdot_k <= cons.Qdot_max)

    # Control bounds
    sigma_max_rad = np.deg2rad(cons.sigma_max)
    opti.subject_to(U >= -sigma_max_rad)
    opti.subject_to(U <=  sigma_max_rad)

    # Control rate & acceleration bounds
    sigma_dot_max = np.deg2rad(cons.sigma_dot_max)
    sigma_ddot_max = np.deg2rad(cons.sigma_ddot_max)

    for k in range(N-1):
        sigma_dot = (U[0,k+1] - U[0,k]) / dt
        opti.subject_to(sigma_dot >= -sigma_dot_max)
        opti.subject_to(sigma_dot <=  sigma_dot_max)

    for k in range(N-2):
        sigma_dot_k  = (U[0,k+1] - U[0,k]) / dt
        sigma_dot_k1 = (U[0,k+2] - U[0,k+1]) / dt
        sigma_ddot = (sigma_dot_k1 - sigma_dot_k) / dt
        opti.subject_to(sigma_ddot >= -sigma_ddot_max)
        opti.subject_to(sigma_ddot <=  sigma_ddot_max)

    # Time step bounds around reachable-set total time
    T_guess = best['t_dep']
    dt_guess = T_guess / N
    opti.subject_to(dt >= 0.5 * dt_guess)
    opti.subject_to(dt <= 1.5 * dt_guess)

    # Objective: maximize h_f, shape γ, center Mach/q in deploy box
    W_ALT   = 1.0
    W_GAM   = 1.0e3
    # W_DEP   = 10.0

    # mach_c = 0.5*(deploy.mach_min + deploy.mach_max)
    # q_c    = 0.5*(deploy.q_min   + deploy.q_max)
    # mach_norm = (mach_f - mach_c) / (0.5*(deploy.mach_max - deploy.mach_min) + 1e-6)
    # q_norm    = (q_f    - q_c   ) / (0.5*(deploy.q_max    - deploy.q_min   ) + 1e-6)

    objective = 0
    objective += -W_ALT * h_f
    objective +=  W_GAM * gam_f**2
    # objective +=  W_DEP * (mach_norm**2 + q_norm**2)

    opti.minimize(objective)

    # Initial guesses from reachable-set bank profile
    sigma1_rad = np.deg2rad(best['sigma1'])
    sigma2_rad = np.deg2rad(best['sigma2'])
    U_guess = np.zeros((1, N))
    X_guess = np.zeros((6, N+1))
    X_guess[:,0] = state0
    dt0 = dt_guess

    for k in range(N):
        t_k = k * dt0
        sigma_k = bank_profile_three_section(t_k, T_guess, sigma1_rad, sigma2_rad)
        U_guess[0,k] = sigma_k
        X_guess[:,k+1] = rk4_step(eom_numpy, X_guess[:,k], sigma_k, dt0)

    opti.set_initial(U, U_guess)
    opti.set_initial(X, X_guess)
    opti.set_initial(dt, dt_guess)

    # Solver settings
    opts = {
        'ipopt.print_level': 3,
        'ipopt.max_iter': 800,
        'ipopt.tol': 1e-5,
        'ipopt.acceptable_tol': 1e-3,
        'ipopt.mu_strategy': 'adaptive',
        'ipopt.nlp_scaling_method': 'gradient-based',
        'print_time': True
    }
    opti.solver('ipopt', opts)

    print("\nStarting optimization with IPOPT...")
    try:
        sol = opti.solve()
        success = True
    except Exception as e:
        print("Optimization failed:", e)
        sol = opti.debug
        success = False

    X_sol = sol.value(X)
    U_val = sol.value(U)
    U_sol = np.array(U_val).flatten()
    dt_sol = float(sol.value(dt))

    return X_sol, U_sol, dt_sol, success

# ============================================================
# 8️⃣ PLOTTING UTILITIES
# ============================================================

def plot_reachable_set(samples, best):
    lat = np.rad2deg([s['lat'] for s in samples])
    lon = np.rad2deg([s['lon'] for s in samples])
    alt = np.array([s['h']/1e3 for s in samples])
    q   = np.array([s['max_q'] for s in samples])
    A   = np.array([s['max_A'] for s in samples])
    Q   = np.array([s['max_Qdot'] for s in samples])

    # Compute downrange and crossrange for each sample using spherical_metrics
    lat_ref = lat0 + np.deg2rad(1.0)  # 1 degree ahead in latitude
    lon_ref = lon0  # Same longitude
    
    RD_all = []
    RC_all = []
    for s in samples:
        lat_f = s['lat']
        lon_f = s['lon']
        _, _, _, RC, RD, RD_go, Rgo = spherical_metrics(
            lat0, lon0, lat_f, lon_f, lat_ref, lon_ref, mars.radius
        )
        RD_all.append(RD / 1e3)  # Convert to km
        RC_all.append(RC / 1e3)  # Convert to km
    
    RD_all = np.array(RD_all)
    RC_all = np.array(RC_all)

    # ===== FIGURE 1: LAT/LON FOOTPRINT =====
    fig1, axes1 = plt.subplots(2, 2, figsize=(14, 10))

    # 1) Altitude contour on lat/lon
    levels_alt = 5
    cf0 = axes1[0,0].tricontourf(lon, lat, alt, levels=levels_alt, cmap='viridis')
    cs0 = axes1[0,0].tricontour(lon, lat, alt, levels=levels_alt, colors='k', linewidths=0.6)
    axes1[0,0].clabel(cs0, fmt="%.1f", fontsize=8)
    axes1[0,0].scatter(np.rad2deg(best['lon']), np.rad2deg(best['lat']),
                       marker='*', s=300, c='red', edgecolors='white', linewidth=1.5,
                       label='Best target', zorder=10)
    cbar0 = fig1.colorbar(cf0, ax=axes1[0,0], label='Altitude [km]')
    axes1[0,0].set_xlabel('Longitude [deg]')
    axes1[0,0].set_ylabel('Latitude [deg]')
    axes1[0,0].set_title('Deployment Altitude Footprint')
    axes1[0,0].grid(True, alpha=0.3)
    axes1[0,0].legend()

    # 2) Maximum dynamic pressure on lat/lon
    levels_q = 12
    q_kPa = q / 1e3  # convert Pa to kPa
    cf1 = axes1[0,1].tricontourf(lon, lat, q_kPa, levels=levels_q, cmap='plasma')
    cs1 = axes1[0,1].tricontour(lon, lat, q_kPa, levels=levels_q, colors='k', linewidths=0.6)
    axes1[0,1].clabel(cs1, fmt="%.1f", fontsize=8)
    cbar1 = fig1.colorbar(cf1, ax=axes1[0,1], label='Max q [kPa]')
    axes1[0,1].text(0.02, 0.98, f'Limit: {cons.q_max/1e3:.1f} kPa', 
                  transform=axes1[0,1].transAxes, fontsize=10, color='white',
                  verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
    axes1[0,1].set_xlabel('Longitude [deg]')
    axes1[0,1].set_ylabel('Latitude [deg]')
    axes1[0,1].set_title('Maximum Dynamic Pressure')
    axes1[0,1].grid(True, alpha=0.3)

    # 3) Maximum acceleration on lat/lon
    levels_a = 12
    cf2 = axes1[1,0].tricontourf(lon, lat, A, levels=levels_a, cmap='hot')
    cs2 = axes1[1,0].tricontour(lon, lat, A, levels=levels_a, colors='k', linewidths=0.6)
    axes1[1,0].clabel(cs2, fmt="%.2f", fontsize=8)
    cbar2 = fig1.colorbar(cf2, ax=axes1[1,0], label='Max A [g]')
    axes1[1,0].text(0.02, 0.98, f'Limit: {cons.A_max:.1f} g', 
                  transform=axes1[1,0].transAxes, fontsize=10, color='white',
                  verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
    axes1[1,0].set_xlabel('Longitude [deg]')
    axes1[1,0].set_ylabel('Latitude [deg]')
    axes1[1,0].set_title('Maximum Acceleration')
    axes1[1,0].grid(True, alpha=0.3)

    # 4) Maximum heat rate on lat/lon
    levels_qd = 12
    Qdot_kW = Q / 1e3  # convert W/m^2 to kW/m^2
    cf3 = axes1[1,1].tricontourf(lon, lat, Qdot_kW, levels=levels_qd, cmap='inferno')
    cs3 = axes1[1,1].tricontour(lon, lat, Qdot_kW, levels=levels_qd, colors='k', linewidths=0.6)
    axes1[1,1].clabel(cs3, fmt="%.1f", fontsize=8)
    cbar3 = fig1.colorbar(cf3, ax=axes1[1,1], label='Max Q̇ [kW/m²]')
    axes1[1,1].text(0.02, 0.98, f'Limit: {cons.Qdot_max/1e3:.0f} kW/m²', 
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
    cf0_dr = axes2[0,0].tricontourf(RD_all, RC_all, alt, levels=levels_alt, cmap='viridis')
    cs0_dr = axes2[0,0].tricontour(RD_all, RC_all, alt, levels=levels_alt, colors='k', linewidths=0.6)
    axes2[0,0].clabel(cs0_dr, fmt="%.1f", fontsize=8)
    axes2[0,0].scatter(best['downrange_km'], best['crossrange_km'],
                       marker='*', s=300, c='red', edgecolors='white', linewidth=1.5,
                       label='Best target', zorder=10)
    cbar0_dr = fig2.colorbar(cf0_dr, ax=axes2[0,0], label='Altitude [km]')
    axes2[0,0].set_xlabel('Downrange [km]')
    axes2[0,0].set_ylabel('Crossrange [km]')
    axes2[0,0].set_title('Deployment Altitude Footprint')
    axes2[0,0].grid(True, alpha=0.3)
    axes2[0,0].legend()

    # 2) Maximum dynamic pressure on downrange/crossrange
    cf1_dr = axes2[0,1].tricontourf(RD_all, RC_all, q_kPa, levels=levels_q, cmap='plasma')
    cs1_dr = axes2[0,1].tricontour(RD_all, RC_all, q_kPa, levels=levels_q, colors='k', linewidths=0.6)
    axes2[0,1].clabel(cs1_dr, fmt="%.1f", fontsize=8)
    cbar1_dr = fig2.colorbar(cf1_dr, ax=axes2[0,1], label='Max q [kPa]')
    axes2[0,1].text(0.02, 0.98, f'Limit: {cons.q_max/1e3:.1f} kPa', 
                  transform=axes2[0,1].transAxes, fontsize=10, color='white',
                  verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
    axes2[0,1].set_xlabel('Downrange [km]')
    axes2[0,1].set_ylabel('Crossrange [km]')
    axes2[0,1].set_title('Maximum Dynamic Pressure')
    axes2[0,1].grid(True, alpha=0.3)

    # 3) Maximum acceleration on downrange/crossrange
    cf2_dr = axes2[1,0].tricontourf(RD_all, RC_all, A, levels=levels_a, cmap='hot')
    cs2_dr = axes2[1,0].tricontour(RD_all, RC_all, A, levels=levels_a, colors='k', linewidths=0.6)
    axes2[1,0].clabel(cs2_dr, fmt="%.2f", fontsize=8)
    cbar2_dr = fig2.colorbar(cf2_dr, ax=axes2[1,0], label='Max A [g]')
    axes2[1,0].text(0.02, 0.98, f'Limit: {cons.A_max:.1f} g', 
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
    axes2[1,1].text(0.02, 0.98, f'Limit: {cons.Qdot_max/1e3:.0f} kW/m²', 
                  transform=axes2[1,1].transAxes, fontsize=10, color='white',
                  verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
    axes2[1,1].set_xlabel('Downrange [km]')
    axes2[1,1].set_ylabel('Crossrange [km]')
    axes2[1,1].set_title('Maximum Heat Rate')
    axes2[1,1].grid(True, alpha=0.3)

    plt.suptitle('Reachable Set: Downrange/Crossrange Footprint', fontsize=14, y=0.995)
    plt.tight_layout()
    plt.show()

def simulate_and_plot_candidate(best, dt=0.25):
    """
    Simulate the selected reachable-set candidate with its normalized bank profile
    and plot state and load histories + ground track.
    """
    sigma1_rad = np.deg2rad(best['sigma1'])
    sigma2_rad = np.deg2rad(best['sigma2'])
    T = best['t_dep']

    t = 0.0
    state = state0.copy()

    t_hist = [t]
    h_hist = [(state[0]-mars.radius)/1e3]
    V_hist = [state[3]]
    gam_hist = [np.rad2deg(state[4])]
    psi_hist = [np.rad2deg(state[5])]
    lat_hist = [np.rad2deg(state[2])]
    lon_hist = [np.rad2deg(state[1])]
    q_hist, A_hist, Qdot_hist = [], [], []
    sigma_hist = []
    mach_hist = []

    while t < T + dt:
        sigma = bank_profile_three_section(t, T, sigma1_rad, sigma2_rad)
        h, q, A, Qdot = aero_loads(state)
        a = mars.sound_speed(h)
        mach = state[3] / max(a, 1e-6)

        q_hist.append(q)
        A_hist.append(A)
        Qdot_hist.append(Qdot)
        sigma_hist.append(np.rad2deg(sigma))
        mach_hist.append(mach)

        state = rk4_step(eom_numpy, state, sigma, dt)
        t += dt

        t_hist.append(t)
        h_hist.append((state[0] - mars.radius)/1e3)
        V_hist.append(state[3])
        gam_hist.append(np.rad2deg(state[4]))
        psi_hist.append(np.rad2deg(state[5]))
        lat_hist.append(np.rad2deg(state[2]))
        lon_hist.append(np.rad2deg(state[1]))

        if deploy_trigger(h, state[3], q) or h <= 0:
            break

    fig, axs = plt.subplots(3, 3, figsize=(15, 10))
    axs = axs.flatten()

    axs[0].plot(t_hist, h_hist); axs[0].set_title('Altitude [km]'); axs[0].grid(True)
    axs[1].plot(t_hist, V_hist); axs[1].set_title('Velocity [m/s]'); axs[1].grid(True)
    axs[2].plot(t_hist, gam_hist); axs[2].set_title('Flight Path Angle [deg]'); axs[2].grid(True)

    axs[3].plot(t_hist, psi_hist); axs[3].set_title('Heading [deg]'); axs[3].grid(True)
    axs[4].plot(t_hist[:-1], sigma_hist); axs[4].set_title('Bank Angle [deg]'); axs[4].grid(True)
    axs[5].plot(t_hist[:-1], mach_hist); axs[5].set_title('Mach'); axs[5].grid(True)
    axs[5].axhspan(deploy.mach_min, deploy.mach_max, alpha=0.2, color='green')

    axs[6].plot(t_hist[:-1], np.array(q_hist)/1e3); axs[6].set_title('Dynamic Pressure [kPa]'); axs[6].grid(True)
    axs[6].axhline(cons.q_max/1e3, ls='--', color='r')
    axs[6].axhspan(deploy.q_min/1e3, deploy.q_max/1e3, alpha=0.2, color='green')

    axs[7].plot(t_hist[:-1], A_hist); axs[7].set_title('Load Factor [g]'); axs[7].grid(True)
    axs[7].axhline(cons.A_max, ls='--', color='r')

    axs[8].plot(t_hist[:-1], np.array(Qdot_hist)/1e3); axs[8].set_title('Heat Rate [kW/m²]'); axs[8].grid(True)
    axs[8].axhline(cons.Qdot_max/1e3, ls='--', color='r')

    plt.suptitle('Selected Reachable-Set Candidate Trajectory', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.show()

    # Ground track
    plt.figure(figsize=(8,6))
    plt.plot(lon_hist, lat_hist, 'b-', lw=2, label='Candidate')
    plt.plot(lon_hist[0], lat_hist[0], 'go', ms=10, label='Entry')
    plt.plot(lon_hist[-1], lat_hist[-1], 'r*', ms=15, label='Deploy')
    plt.xlabel('Longitude [deg]')
    plt.ylabel('Latitude [deg]')
    plt.title('Ground Track (Candidate)', fontsize=13, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

def plot_optimized_trajectory(X, U, dt):
    t = np.arange(X.shape[1]) * dt
    r   = X[0,:]
    lon = X[1,:]
    lat = X[2,:]
    V   = X[3,:]
    gam = X[4,:]
    psi = X[5,:]
    h   = (r - mars.radius)/1e3
    sigma = np.append(U, U[-1])

    # compute loads + Mach
    q_hist, A_hist, Qdot_hist, mach_hist = [], [], [], []
    for i in range(len(t)):
        state_i = X[:,i]
        h_i = state_i[0] - mars.radius
        V_i = state_i[3]
        rho_i = mars.density(h_i)
        D_i = 0.5 * rho_i * V_i**2 / veh.beta
        L_i = veh.L_over_D * D_i
        q_i = 0.5 * rho_i * V_i**2
        A_i = np.sqrt(D_i**2 + L_i**2) / 9.80665
        Qdot_i = veh.k_heat_flux * (rho_i**veh.N) * (V_i**veh.M)
        a_i = mars.sound_speed(h_i)
        mach_i = V_i / max(a_i, 1e-6)
        q_hist.append(q_i); A_hist.append(A_i); Qdot_hist.append(Qdot_i); mach_hist.append(mach_i)

    q_hist = np.array(q_hist); A_hist = np.array(A_hist)
    Qdot_hist = np.array(Qdot_hist); mach_hist = np.array(mach_hist)

    # sigma derivatives
    sigma_dot = np.gradient(sigma, dt)
    sigma_ddot = np.gradient(sigma_dot, dt)

    fig, axs = plt.subplots(4, 3, figsize=(16, 12))
    axs = axs.flatten()

    axs[0].plot(t, h); axs[0].set_title('Altitude [km]'); axs[0].grid(True)
    axs[1].plot(t, V); axs[1].set_title('Velocity [m/s]'); axs[1].grid(True)
    axs[2].plot(t, np.rad2deg(gam)); axs[2].set_title('Flight Path Angle [deg]'); axs[2].grid(True)

    axs[3].plot(t, np.rad2deg(psi)); axs[3].set_title('Heading [deg]'); axs[3].grid(True)
    axs[4].plot(t, np.rad2deg(sigma)); axs[4].set_title('Bank Angle [deg]'); axs[4].grid(True)
    axs[4].axhline(cons.sigma_max, ls='--', color='r'); axs[4].axhline(-cons.sigma_max, ls='--', color='r')

    axs[5].plot(t, mach_hist); axs[5].set_title('Mach'); axs[5].grid(True)
    axs[5].axhspan(deploy.mach_min, deploy.mach_max, alpha=0.2, color='green')

    axs[6].plot(t, q_hist/1e3); axs[6].set_title('Dynamic Pressure [kPa]'); axs[6].grid(True)
    axs[6].axhline(cons.q_max/1e3, ls='--', color='r')
    axs[6].axhspan(deploy.q_min/1e3, deploy.q_max/1e3, alpha=0.2, color='green')

    axs[7].plot(t, A_hist); axs[7].set_title('Load [g]'); axs[7].grid(True)
    axs[7].axhline(cons.A_max, ls='--', color='r')

    axs[8].plot(t, Qdot_hist/1e3); axs[8].set_title('Heat Rate [kW/m²]'); axs[8].grid(True)
    axs[8].axhline(cons.Qdot_max/1e3, ls='--', color='r')

    axs[9].plot(t, np.rad2deg(sigma_dot)); axs[9].set_title('Bank Rate [deg/s]'); axs[9].grid(True)
    axs[9].axhline(cons.sigma_dot_max, ls='--', color='r')
    axs[9].axhline(-cons.sigma_dot_max, ls='--', color='r')

    axs[10].plot(t, np.rad2deg(sigma_ddot)); axs[10].set_title('Bank Accel [deg/s²]'); axs[10].grid(True)
    axs[10].axhline(cons.sigma_ddot_max, ls='--', color='r')
    axs[10].axhline(-cons.sigma_ddot_max, ls='--', color='r')

    plt.suptitle('Optimized Trajectory: States, Loads, Controls', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.show()

    # Ground track
    plt.figure(figsize=(8,6))
    plt.scatter(np.rad2deg(lon), np.rad2deg(lat), c=h, cmap='viridis', s=40, edgecolors='k', linewidth=0.5)
    plt.plot(np.rad2deg(lon), np.rad2deg(lat), 'k-', alpha=0.3)
    plt.scatter(np.rad2deg(lon[0]), np.rad2deg(lat[0]), c='green', s=80, label='Entry')
    plt.scatter(np.rad2deg(lon[-1]), np.rad2deg(lat[-1]), c='red', s=80, marker='*', label='Deploy')
    plt.xlabel('Longitude [deg]'); plt.ylabel('Latitude [deg]')
    plt.title('Optimized Ground Track (colored by altitude)', fontsize=13, fontweight='bold')
    plt.grid(True, alpha=0.3); plt.legend()
    cbar = plt.colorbar(); cbar.set_label('Altitude [km]')
    plt.tight_layout()
    plt.show()

# ============================================================
# 9️⃣ MAIN
# ============================================================

def main():
    print("\n================ REACHABLE SET =================")
    samples = build_reachable_set(n_grid=9, dt=0.25, tmax=4000.0)
    if not samples:
        print("No valid trajectories in reachable set. Aborting.")
        return

    best = select_best_target(samples, max_crossrange_km=5.0)
    if best is None:
        print("No valid target found. Aborting.")
        return

    plot_reachable_set(samples, best)
    simulate_and_plot_candidate(best)

    print("\n================ OPTIMIZATION =================")
    X_sol, U_sol, dt_sol, success = optimize_to_target(best, N=180)

    if not success:
        print("Warning: optimization may not have fully converged (debug solution).")

    # ============================================================
    # COMPREHENSIVE COMPARISON: REACHABLE vs OPTIMIZED
    # ============================================================
    print("\n" + "="*80)
    print("COMPREHENSIVE TRAJECTORY COMPARISON")
    print("="*80)
    
    # Extract final state from optimized trajectory
    r_opt = X_sol[0, -1]
    lon_opt = X_sol[1, -1]
    lat_opt = X_sol[2, -1]
    V_opt = X_sol[3, -1]
    gam_opt = X_sol[4, -1]
    psi_opt = X_sol[5, -1]
    h_opt = r_opt - mars.radius
    
    # Compute final loads for optimized trajectory
    rho_opt = mars.density(h_opt)
    D_opt = 0.5 * rho_opt * V_opt**2 / veh.beta
    L_opt = veh.L_over_D * D_opt
    q_opt = 0.5 * rho_opt * V_opt**2
    A_opt = np.sqrt(D_opt**2 + L_opt**2) / 9.80665
    Qdot_opt = veh.k_heat_flux * (rho_opt**veh.N) * (V_opt**veh.M)
    a_opt = mars.sound_speed(h_opt)
    mach_opt = V_opt / max(a_opt, 1e-6)
    
    # Compute trajectory duration and max loads for optimized
    t_total_opt = dt_sol * len(U_sol)
    max_q_opt = 0.0
    max_A_opt = 0.0
    max_Qdot_opt = 0.0
    for i in range(X_sol.shape[1]):
        h_i = X_sol[0, i] - mars.radius
        V_i = X_sol[3, i]
        rho_i = mars.density(h_i)
        D_i = 0.5 * rho_i * V_i**2 / veh.beta
        L_i = veh.L_over_D * D_i
        q_i = 0.5 * rho_i * V_i**2
        A_i = np.sqrt(D_i**2 + L_i**2) / 9.80665
        Qdot_i = veh.k_heat_flux * (rho_i**veh.N) * (V_i**veh.M)
        max_q_opt = max(max_q_opt, q_i)
        max_A_opt = max(max_A_opt, A_i)
        max_Qdot_opt = max(max_Qdot_opt, Qdot_i)
    
    # Extract reachable trajectory data
    h_reach = best['h']
    lat_reach = best['lat']
    lon_reach = best['lon']
    V_reach = best['V']
    gam_reach = best['gam']
    psi_reach = best['psi']
    t_total_reach = best['t_dep']
    max_q_reach = best['max_q']
    max_A_reach = best['max_A']
    max_Qdot_reach = best['max_Qdot']
    q_reach = best['q_dep']
    A_reach = best['A_dep']
    Qdot_reach = best['Qdot_dep']
    mach_reach = best['mach_dep']
    
    # Compute position difference using spherical_metrics
    lat_ref = lat0 + np.deg2rad(1.0)
    lon_ref = lon0
    
    # Compute metrics for reachable trajectory
    _, _, _, RC_reach, RD_reach, _, _ = spherical_metrics(
        lat0, lon0, lat_reach, lon_reach, lat_ref, lon_ref, mars.radius
    )
    
    # Compute metrics for optimized trajectory
    _, _, _, RC_opt, RD_opt, _, _ = spherical_metrics(
        lat0, lon0, lat_opt, lon_opt, lat_ref, lon_ref, mars.radius
    )
    
    # Position difference between optimized and reachable
    dist_diff = great_circle_distance(lat_reach, lon_reach, lat_opt, lon_opt, mars.radius)
    RC_diff = RC_opt - RC_reach
    
    print("\n" + "-"*80)
    print("FINAL DEPLOYMENT STATE COMPARISON")
    print("-"*80)
    print(f"{'Metric':<30} {'Reachable':<20} {'Optimized':<20} {'Difference':<20}")
    print("-"*80)
    
    # Altitude
    h_diff_km = (h_opt - h_reach) / 1e3
    h_diff_pct = 100 * (h_opt - h_reach) / h_reach
    print(f"{'Altitude [km]':<30} {h_reach/1e3:>19.3f} {h_opt/1e3:>19.3f} {h_diff_km:>+18.3f} ({h_diff_pct:+.2f}%)")
    
    # Latitude
    lat_diff_deg = np.rad2deg(lat_opt - lat_reach)
    print(f"{'Latitude [deg]':<30} {np.rad2deg(lat_reach):>19.4f} {np.rad2deg(lat_opt):>19.4f} {lat_diff_deg:>+18.4f}")
    
    # Longitude
    lon_diff_deg = np.rad2deg(lon_opt - lon_reach)
    print(f"{'Longitude [deg]':<30} {np.rad2deg(lon_reach):>19.4f} {np.rad2deg(lon_opt):>19.4f} {lon_diff_deg:>+18.4f}")
    
    # Crossrange
    RC_diff_pct = 100 * (RC_opt - RC_reach) / abs(RC_reach) if abs(RC_reach) > 1e-3 else 0.0
    print(f"{'Crossrange [km]':<30} {RC_reach/1e3:>19.3f} {RC_opt/1e3:>19.3f} {RC_diff/1e3:>+18.3f} ({RC_diff_pct:+.2f}%)")
    
    # Downrange
    RD_diff = RD_opt - RD_reach
    RD_diff_pct = 100 * (RD_opt - RD_reach) / RD_reach
    print(f"{'Downrange [km]':<30} {RD_reach/1e3:>19.3f} {RD_opt/1e3:>19.3f} {RD_diff/1e3:>+18.3f} ({RD_diff_pct:+.2f}%)")
    
    # Position miss distance
    print(f"{'Position Miss Distance [km]':<30} {'(reference)':>19} {'(from reach)':>19} {dist_diff/1e3:>19.3f}")
    
    # Mach number
    mach_diff = mach_opt - mach_reach
    mach_diff_pct = 100 * (mach_opt - mach_reach) / mach_reach
    print(f"{'Mach Number':<30} {mach_reach:>19.3f} {mach_opt:>19.3f} {mach_diff:>+18.3f} ({mach_diff_pct:+.2f}%)")
    
    # Dynamic pressure
    q_diff = q_opt - q_reach
    q_diff_pct = 100 * (q_opt - q_reach) / q_reach
    print(f"{'Dynamic Pressure [Pa]':<30} {q_reach:>19.1f} {q_opt:>19.1f} {q_diff:>+18.1f} ({q_diff_pct:+.2f}%)")
    
    # Acceleration
    A_diff = A_opt - A_reach
    A_diff_pct = 100 * (A_opt - A_reach) / A_reach
    print(f"{'Deceleration [g]':<30} {A_reach:>19.3f} {A_opt:>19.3f} {A_diff:>+18.3f} ({A_diff_pct:+.2f}%)")
    
    # Heat rate
    Qdot_diff = Qdot_opt - Qdot_reach
    Qdot_diff_pct = 100 * (Qdot_opt - Qdot_reach) / Qdot_reach
    print(f"{'Heat Rate [W/m²]':<30} {Qdot_reach:>19.1f} {Qdot_opt:>19.1f} {Qdot_diff:>+18.1f} ({Qdot_diff_pct:+.2f}%)")
    
    # Velocity
    V_diff = V_opt - V_reach
    V_diff_pct = 100 * (V_opt - V_reach) / V_reach
    print(f"{'Velocity [m/s]':<30} {V_reach:>19.2f} {V_opt:>19.2f} {V_diff:>+18.2f} ({V_diff_pct:+.2f}%)")
    
    # Flight path angle
    gam_diff = np.rad2deg(gam_opt - gam_reach)
    gam_diff_pct = 100 * (gam_opt - gam_reach) / gam_reach
    print(f"{'Flight Path Angle [deg]':<30} {np.rad2deg(gam_reach):>19.3f} {np.rad2deg(gam_opt):>19.3f} {gam_diff:>+18.3f} ({gam_diff_pct:+.2f}%)")
    
    # Heading angle
    psi_diff = np.rad2deg(psi_opt - psi_reach)
    psi_diff_pct = 100 * (psi_opt - psi_reach) / psi_reach if abs(psi_reach) > 1e-6 else 0.0
    print(f"{'Heading Angle [deg]':<30} {np.rad2deg(psi_reach):>19.3f} {np.rad2deg(psi_opt):>19.3f} {psi_diff:>+18.3f} ({psi_diff_pct:+.2f}%)")
    
    print("\n" + "-"*80)
    print("TRAJECTORY HISTORY COMPARISON")
    print("-"*80)
    print(f"{'Metric':<30} {'Reachable':<20} {'Optimized':<20} {'Difference':<20}")
    print("-"*80)
    
    # Duration
    t_diff = t_total_opt - t_total_reach
    t_diff_pct = 100 * (t_total_opt - t_total_reach) / t_total_reach
    print(f"{'Trajectory Duration [s]':<30} {t_total_reach:>19.2f} {t_total_opt:>19.2f} {t_diff:>+18.2f} ({t_diff_pct:+.2f}%)")
    
    # Max dynamic pressure
    max_q_diff = max_q_opt - max_q_reach
    max_q_diff_pct = 100 * (max_q_opt - max_q_reach) / max_q_reach
    print(f"{'Max Dynamic Pressure [Pa]':<30} {max_q_reach:>19.1f} {max_q_opt:>19.1f} {max_q_diff:>+18.1f} ({max_q_diff_pct:+.2f}%)")
    
    # Max acceleration
    max_A_diff = max_A_opt - max_A_reach
    max_A_diff_pct = 100 * (max_A_opt - max_A_reach) / max_A_reach
    print(f"{'Max Deceleration [g]':<30} {max_A_reach:>19.3f} {max_A_opt:>19.3f} {max_A_diff:>+18.3f} ({max_A_diff_pct:+.2f}%)")
    
    # Max heat rate
    max_Qdot_diff = max_Qdot_opt - max_Qdot_reach
    max_Qdot_diff_pct = 100 * (max_Qdot_opt - max_Qdot_reach) / max_Qdot_reach
    print(f"{'Max Heat Rate [W/m²]':<30} {max_Qdot_reach:>19.1f} {max_Qdot_opt:>19.1f} {max_Qdot_diff:>+18.1f} ({max_Qdot_diff_pct:+.2f}%)")
    
    print("\n" + "-"*80)
    print("DEPLOYMENT ENVELOPE COMPLIANCE")
    print("-"*80)
    print(f"{'Constraint':<30} {'Limit':<20} {'Reachable':<20} {'Optimized':<20}")
    print("-"*80)
    
    # Mach range
    mach_reach_ok = "✓" if deploy.mach_min <= mach_reach <= deploy.mach_max else "✗"
    mach_opt_ok = "✓" if deploy.mach_min <= mach_opt <= deploy.mach_max else "✗"
    print(f"{'Mach Number':<30} {f'[{deploy.mach_min:.1f}, {deploy.mach_max:.1f}]':<20} {f'{mach_reach:.2f} {mach_reach_ok}':<20} {f'{mach_opt:.2f} {mach_opt_ok}':<20}")
    
    # Dynamic pressure range
    q_reach_ok = "✓" if deploy.q_min <= q_reach <= deploy.q_max else "✗"
    q_opt_ok = "✓" if deploy.q_min <= q_opt <= deploy.q_max else "✗"
    print(f"{'Dynamic Pressure [Pa]':<30} {f'[{deploy.q_min:.0f}, {deploy.q_max:.0f}]':<20} {f'{q_reach:.0f} {q_reach_ok}':<20} {f'{q_opt:.0f} {q_opt_ok}':<20}")
    
    # Altitude minimum
    h_reach_ok = "✓" if h_reach >= deploy.h_min else "✗"
    h_opt_ok = "✓" if h_opt >= deploy.h_min else "✗"
    print(f"{'Altitude [m]':<30} {f'>= {deploy.h_min:.0f}':<20} {f'{h_reach:.0f} {h_reach_ok}':<20} {f'{h_opt:.0f} {h_opt_ok}':<20}")
    
    print("\n" + "-"*80)
    print("PATH CONSTRAINT COMPLIANCE")
    print("-"*80)
    print(f"{'Constraint':<30} {'Limit':<20} {'Reachable Max':<20} {'Optimized Max':<20}")
    print("-"*80)
    
    # Dynamic pressure limit
    max_q_reach_ok = "✓" if max_q_reach <= cons.q_max else "✗"
    max_q_opt_ok = "✓" if max_q_opt <= cons.q_max else "✗"
    print(f"{'Dynamic Pressure [Pa]':<30} {f'<= {cons.q_max:.0f}':<20} {f'{max_q_reach:.0f} {max_q_reach_ok}':<20} {f'{max_q_opt:.0f} {max_q_opt_ok}':<20}")
    
    # Acceleration limit
    max_A_reach_ok = "✓" if max_A_reach <= cons.A_max else "✗"
    max_A_opt_ok = "✓" if max_A_opt <= cons.A_max else "✗"
    print(f"{'Deceleration [g]':<30} {f'<= {cons.A_max:.1f}':<20} {f'{max_A_reach:.2f} {max_A_reach_ok}':<20} {f'{max_A_opt:.2f} {max_A_opt_ok}':<20}")
    
    # Heat rate limit
    max_Qdot_reach_ok = "✓" if max_Qdot_reach <= cons.Qdot_max else "✗"
    max_Qdot_opt_ok = "✓" if max_Qdot_opt <= cons.Qdot_max else "✗"
    print(f"{'Heat Rate [W/m²]':<30} {f'<= {cons.Qdot_max:.0f}':<20} {f'{max_Qdot_reach:.0f} {max_Qdot_reach_ok}':<20} {f'{max_Qdot_opt:.0f} {max_Qdot_opt_ok}':<20}")
    
    print("\n" + "-"*80)
    print("CONTROL STRATEGY COMPARISON")
    print("-"*80)
    
    print(f"\nReachable Set Strategy:")
    print(f"  Bank Profile: Three-section (σ1={best['sigma1']:.2f}°, σ2={best['sigma2']:.2f}°)")
    print(f"  Section 1 (0-35%): Constant σ1 = {best['sigma1']:.2f}°")
    print(f"  Section 2 (35-50%): Linear transition to σ2")
    print(f"  Section 3 (50-100%): Constant σ2 = {best['sigma2']:.2f}°")
    
    sigma_opt_mean = np.rad2deg(np.mean(U_sol))
    sigma_opt_std = np.rad2deg(np.std(U_sol))
    sigma_opt_max = np.rad2deg(np.max(U_sol))
    sigma_opt_min = np.rad2deg(np.min(U_sol))
    
    print(f"\nOptimized Strategy:")
    print(f"  Bank Angle Statistics:")
    print(f"    Mean:   {sigma_opt_mean:>7.2f}°")
    print(f"    Std:    {sigma_opt_std:>7.2f}°")
    print(f"    Min:    {sigma_opt_min:>7.2f}°")
    print(f"    Max:    {sigma_opt_max:>7.2f}°")
    print(f"    Range:  [{sigma_opt_min:.2f}°, {sigma_opt_max:.2f}°]")
    
    print("\n" + "-"*80)
    print("SUMMARY")
    print("-"*80)
    
    # Key improvements
    if h_diff_km > 0:
        print(f"✓ Altitude IMPROVED by {h_diff_km:.3f} km ({h_diff_pct:+.2f}%)")
    else:
        print(f"✗ Altitude DECREASED by {abs(h_diff_km):.3f} km ({h_diff_pct:.2f}%)")
    
    if abs(mach_opt - 1.8) < abs(mach_reach - 1.8):  # 1.8 is center of [1.4, 2.2]
        print(f"✓ Mach number MORE CENTERED in deployment envelope")
    else:
        print(f"○ Mach number less centered in deployment envelope")
    
    if abs(q_opt - 550) < abs(q_reach - 550):  # 550 is center of [300, 800]
        print(f"✓ Dynamic pressure MORE CENTERED in deployment envelope")
    else:
        print(f"○ Dynamic pressure less centered in deployment envelope")
    
    if max_q_opt < max_q_reach:
        print(f"✓ Peak dynamic pressure REDUCED by {abs(max_q_diff):.1f} Pa ({abs(max_q_diff_pct):.2f}%)")
    elif max_q_opt > max_q_reach:
        print(f"✗ Peak dynamic pressure INCREASED by {max_q_diff:.1f} Pa ({max_q_diff_pct:.2f}%)")
    
    if max_A_opt < max_A_reach:
        print(f"✓ Peak acceleration REDUCED by {abs(max_A_diff):.3f} g ({abs(max_A_diff_pct):.2f}%)")
    elif max_A_opt > max_A_reach:
        print(f"✗ Peak acceleration INCREASED by {max_A_diff:.3f} g ({max_A_diff_pct:.2f}%)")
    
    if max_Qdot_opt < max_Qdot_reach:
        print(f"✓ Peak heat rate REDUCED by {abs(max_Qdot_diff):.1f} W/m² ({abs(max_Qdot_diff_pct):.2f}%)")
    elif max_Qdot_opt > max_Qdot_reach:
        print(f"✗ Peak heat rate INCREASED by {max_Qdot_diff:.1f} W/m² ({max_Qdot_diff_pct:.2f}%)")
    
    print(f"\nPosition Miss: {dist_diff/1e3:.3f} km (lateral: {RC_diff/1e3:.3f} km)")
    print(f"Time Difference: {abs(t_diff):.2f} s ({abs(t_diff_pct):.2f}%)")
    
    print("="*80 + "\n")

    plot_optimized_trajectory(X_sol, U_sol, dt_sol)
    print("\n✅ Integrated optimization complete.")

if __name__ == "__main__":
    main()



