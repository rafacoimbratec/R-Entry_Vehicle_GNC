import numpy as np
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple, Dict, Any
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from spherical_metrics import spherical_metrics

# ============================================================
# MODELS & CONSTANTS
# ============================================================
@dataclass
class Mars:
    radius: float = 3396.2e3            # [m]
    mu: float = 4.282837e13             # [m^3/s^2]
    omega: float = 7.088e-5             # [rad/s]
    rho0: float = 0.020                 # [kg/m^3] effective ref density
    Hs: float = 11100.0                 # [m] scale height
    # Gas properties (CO2-dominated)
    gamma_gas: float = 1.294
    R_gas: float = 188.9                # [J/(kg·K)]

    def temperature(self, h: float) -> float:
        """Atmospheric temperature model (simplified; altitude-dependent)."""
        h_km = h / 1e3
        T = 1.4e-13 * h_km**3 - 8.85e-9 * h_km**2 - 1.245e-3 * h_km + 205.36
        return T

    def sound_speed(self, h: float) -> float:
        """Local speed of sound [m/s] using temperature(h)."""
        T = max(self.temperature(h), 1.0)
        return np.sqrt(self.gamma_gas * self.R_gas * T)


@dataclass
class Vehicle:
    beta: float = 135.0                 # ballistic coefficient [kg/m^2]
    L_over_D: float = 0.24              # lift-to-drag

mars = Mars()
veh  = Vehicle()

# ============================================================
# 3DOF EOM + RK4
# ============================================================
def eom(state, sigma):
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
    k1 = f(state, sigma)
    k2 = f(state + 0.5*dt*k1, sigma)
    k3 = f(state + 0.5*dt*k2, sigma)
    k4 = f(state + dt*k3, sigma)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

# ============================================================
# NLP CONFIGURATION
# ============================================================
DT = 0.5  # Integration timestep [s]
T_END = 400.0  # Maximum trajectory duration [s]

# Initial conditions
deg = np.deg2rad
h0   = 125e3
r0   = mars.radius + h0
V0   = 4700.0
gam0 = deg(-12.0)
psi0 = deg(-2.8758)
lat0 = deg(-21.5)
lon0 = deg(-176)
lat_target = deg(-7)
lon_target = deg(-176.5)
state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])

# Bank angle limits
SIGMA_MIN = deg(-90.0)
SIGMA_MAX = deg(90.0)
SIGMA_DOT_MAX = deg(20.0)      # [rad/s]
SIGMA_DDOT_MAX = deg(5.0)      # [rad/s²]

# NLP discretization: control points for sigma(tau), where tau is normalized trajectory progress [0,1]
N_CONTROL_POINTS = 4  # Number of control points for σ(τ) - fewer points for bang-bang behavior
TAU_CONTROL = np.linspace(0, 1, N_CONTROL_POINTS)  # Normalized time from 0 to 1

# Target range-to-go constraint (instead of d=0)
RGO_TARGET = 5.0e3  # [m] - target Rgo at deployment = 5 km
RGO_TOLERANCE = 2.0e3  # [m] - tolerance band for Rgo constraint (5 ± 2 km)

# Bang-bang incentive parameters
BANG_BANG_WEIGHT = 1e-1  # Weight for encouraging extreme bank angles (start small!)
SMOOTH_PENALTY_WEIGHT = 0.0  # Weight for penalizing smooth transitions (0 = allow sharp reversals)
INTERP_KIND = 'previous'  # 'linear' for smooth or 'previous' for piecewise-constant (more bang-bang)
USE_RGO_PENALTY = False  # If True, use soft penalty instead of hard constraint for Rgo
USE_RATE_CONSTRAINTS = False  # If True, enforce sigma_dot and sigma_ddot limits (may prevent convergence)

# ============================================================
# TRAJECTORY SIMULATION
# ============================================================
def simulate_trajectory(sigma_control_points, return_history=False):
    """
    Simulate trajectory with given control points for sigma(tau).
    
    Args:
        sigma_control_points: array of length N_CONTROL_POINTS, sigma values in radians
                             indexed by normalized time tau in [0,1]
        return_history: if True, return detailed trajectory history
    
    Returns:
        final_state, deploy_occurred, history (if requested)
    
    Note: sigma is parameterized on normalized time tau = t/t_final, so all control
          points are actively used regardless of when the trajectory ends.
    """
    # First pass: quick simulation to find trajectory duration
    # (we need to know t_final to map tau -> t)
    t_temp = 0.0
    state_temp = state0.copy()
    
    # Create temporary interpolator assuming T_END as duration
    sigma_interp_temp = interp1d(TAU_CONTROL * T_END, sigma_control_points, kind=INTERP_KIND,
                                  bounds_error=False, fill_value=(sigma_control_points[0], sigma_control_points[-1]))
    
    # Quick run to find actual trajectory end time
    while t_temp < T_END:
        r_temp, _, _, V_temp, _, _ = state_temp
        h_temp = r_temp - mars.radius
        
        if h_temp <= 0.0:
            t_final = t_temp
            break
        
        rho_temp = mars.rho0 * np.exp(-(r_temp - mars.radius) / mars.Hs)
        q_temp = 0.5 * rho_temp * V_temp**2
        a_sound_temp = mars.sound_speed(h_temp)
        mach_temp = V_temp / max(a_sound_temp, 1e-6)
        
        if (1.4 <= mach_temp <= 2.2) and (300.0 <= q_temp <= 800.0):
            t_final = t_temp
            break
        
        sigma_temp = float(sigma_interp_temp(t_temp))
        state_temp = rk4_step(eom, state_temp, sigma_temp, DT)
        t_temp += DT
    else:
        t_final = T_END
    
    # Now create the proper interpolator: tau * t_final -> sigma
    # This ensures all control points span the actual trajectory
    t_control_actual = TAU_CONTROL * t_final
    sigma_interp = interp1d(t_control_actual, sigma_control_points, kind=INTERP_KIND,
                            bounds_error=False, fill_value=(sigma_control_points[0], sigma_control_points[-1]))
    
    t = 0.0
    state = state0.copy()
    deployed = False
    deploy_time = None
    
    if return_history:
        hist = {
            "t": [], "h": [], "V": [], "gam": [], "psi": [], "lat": [], "lon": [],
            "rho": [], "q": [], "mach": [], "sigma": [],
            "Rgo": [], "RC": [], "RD": [], "RD_go": []
        }
    
    while t < T_END:
        r, lon, lat, V, gam, psi = state
        h = r - mars.radius
        
        # Check termination conditions
        if h <= 0.0:  # Impact
            break
            
        # Check deployment condition
        rho = mars.rho0 * np.exp(-(r - mars.radius) / mars.Hs)
        q = 0.5 * rho * V**2
        a_sound = mars.sound_speed(h)
        mach = V / max(a_sound, 1e-6)
        
        deploy_cond = (1.4 <= mach <= 2.2) and (300.0 <= q <= 800.0)
        if deploy_cond and not deployed:
            deployed = True
            deploy_time = t
            break
        
        # Get current sigma from interpolator
        sigma = float(sigma_interp(t))
        
        # Record history
        if return_history:
            # Compute spherical metrics
            _, _, _, RC, RD, RD_go, Rgo = spherical_metrics(
                lat0, lon0, lat, lon, lat_target, lon_target, mars.radius
            )
            hist["t"].append(t)
            hist["h"].append(h)
            hist["V"].append(V)
            hist["gam"].append(gam)
            hist["psi"].append(psi)
            hist["lat"].append(lat)
            hist["lon"].append(lon)
            hist["rho"].append(rho)
            hist["q"].append(q)
            hist["mach"].append(mach)
            hist["sigma"].append(sigma)
            hist["Rgo"].append(Rgo)
            hist["RC"].append(RC)
            hist["RD"].append(RD)
            hist["RD_go"].append(RD_go)
        
        # Integrate dynamics
        state = rk4_step(eom, state, sigma, DT)
        t += DT
    
    final_state = state
    
    if return_history:
        return final_state, deployed, deploy_time, hist
    else:
        return final_state, deployed, deploy_time

# ============================================================
# NLP OBJECTIVE AND CONSTRAINTS
# ============================================================
class OptimizationCounter:
    def __init__(self):
        self.obj_count = 0
        self.cons_count = 0
        self.start_time = time.perf_counter()

counter = OptimizationCounter()

def objective(sigma_control_points):
    """
    Objective: MAXIMIZE final altitude at deployment (minimize negative altitude).
    Added incentive for bang-bang control (extreme bank angles).
    Optionally includes soft penalty for Rgo constraint.
    """
    counter.obj_count += 1
    
    final_state, deployed, deploy_time = simulate_trajectory(sigma_control_points)
    r_final = final_state[0]
    h_final = r_final - mars.radius
    _, lon_final, lat_final, _, _, _ = final_state
    
    # Compute Rgo at deployment
    _, _, _, RC, RD, RD_go, Rgo = spherical_metrics(
        lat0, lon0, lat_final, lon_final, lat_target, lon_target, mars.radius
    )
    
    # Primary objective: maximize altitude
    J_altitude = -h_final
    
    # Rgo penalty (if using soft constraint instead of hard)
    if USE_RGO_PENALTY:
        # Penalize deviation from target Rgo
        rgo_error = abs(Rgo - RGO_TARGET)
        rgo_penalty = 1e3 * rgo_error  # Heavy penalty for Rgo deviation
    else:
        rgo_penalty = 0.0
    
    # Bang-bang incentive: encourage bank angles near ±90°
    # Compute "distance from extreme" for each control point
    sigma_normalized = sigma_control_points / SIGMA_MAX  # normalize to [-1, 1]
    bang_bang_penalty = BANG_BANG_WEIGHT * np.sum(1.0 - np.abs(sigma_normalized)**2)
    # This penalizes moderate angles; rewards being near ±1 (±90°)
    
    # Optional: penalize smooth regions (encourage sharp transitions)
    # Small penalty if consecutive points are similar (want discontinuities)
    if SMOOTH_PENALTY_WEIGHT > 0:
        smoothness = np.sum(np.abs(np.diff(sigma_control_points)))
        smooth_penalty = -SMOOTH_PENALTY_WEIGHT * smoothness  # negative = reward roughness
    else:
        smooth_penalty = 0.0
    
    J = J_altitude #+ rgo_penalty #+ bang_bang_penalty + smooth_penalty
    
    if counter.obj_count % 10 == 0:
        elapsed = time.perf_counter() - counter.start_time
        #print(f"[OBJ] eval={counter.obj_count:4d}  h={h_final/1e3:7.2f}km  "
              #f"Rgo={Rgo/1e3:6.2f}km RC={RC/1e3:6.2f}km RD={RD/1e3:6.2f}km  RD_go={RD_go/1e3:6.2f}km J={J:10.7f} elapsed={elapsed:6.1f}s")
    
    return J

def rgo_constraint(sigma_control_points):
    """
    Constraint: Rgo at deployment = 5 km (equality constraint).
    Returns: Rgo - RGO_TARGET (should be driven to 0)
    """
    counter.cons_count += 1
    
    final_state, deployed, deploy_time = simulate_trajectory(sigma_control_points)
    _, lon_final, lat_final, _, _, _ = final_state
    
    # Compute Rgo at deployment
    _, _, _, RC, RD, RD_go, Rgo = spherical_metrics(
        lat0, lon0, lat_final, lon_final, lat_target, lon_target, mars.radius
    )
    
    # Constraint: Rgo should equal RGO_TARGET
    constraint_value = Rgo - RGO_TARGET
    
    #if counter.cons_count % 10 == 0:
        #print(f"[CONS] eval={counter.cons_count:4d}  Rgo={Rgo/1e3:7.2f} km  target={RGO_TARGET/1e3:.2f} km")
    
    return constraint_value

def deployment_constraint(sigma_control_points):
    """
    Constraint: must deploy (not impact).
    Returns: 0 if deployed, negative if not deployed
    """
    final_state, deployed, deploy_time = simulate_trajectory(sigma_control_points)
    
    # Return 0 if deployed (satisfied), -1 if not (violated)
    return 0.0 if deployed else -1.0

# ============================================================
# RATE/ACCELERATION CONSTRAINTS
# ============================================================
def rate_constraints(sigma_control_points):
    """
    Ensure sigma_dot and sigma_ddot stay within limits based on normalized time.
    Returns array of constraint violations (should all be >= 0).
    
    Note: Since control points are in normalized time tau, we need to account for
    the actual trajectory duration when computing rates. We'll estimate rates
    conservatively assuming the minimum reasonable trajectory time.
    """
    # Estimate minimum trajectory duration (typically 100-200s for this problem)
    # This gives us a conservative constraint
    t_min_estimate = 100.0  # [s] - conservative minimum trajectory duration
    
    # dt in normalized time space
    dtau = TAU_CONTROL[1] - TAU_CONTROL[0]
    
    # Convert to physical time spacing
    dt_physical = dtau * t_min_estimate
    
    constraints = []
    
    # Rate constraints: |sigma_dot| <= SIGMA_DOT_MAX
    for i in range(len(sigma_control_points) - 1):
        sigma_dot = (sigma_control_points[i+1] - sigma_control_points[i]) / dt_physical
        constraints.append(SIGMA_DOT_MAX - abs(sigma_dot))
    
    # Acceleration constraints: |sigma_ddot| <= SIGMA_DDOT_MAX
    for i in range(len(sigma_control_points) - 2):
        sigma_ddot = (sigma_control_points[i+2] - 2*sigma_control_points[i+1] + sigma_control_points[i]) / dt_physical**2
        constraints.append(SIGMA_DDOT_MAX - abs(sigma_ddot))
    
    return np.array(constraints)

# ============================================================
# OPTIMIZATION SETUP
# ============================================================
def solve_nlp():
    """
    Solve the NLP problem: minimize -h_final subject to distance = 0.
    """
    print("\n" + "="*70)
    print("NONLINEAR PROGRAMMING OPTIMIZATION")
    print("="*70)
    print(f"Control points: {N_CONTROL_POINTS}")
    print(f"Time horizon: {T_END} s")
    print(f"Integration timestep: {DT} s")
    print(f"Target: lat={np.rad2deg(lat_target):.3f}°, lon={np.rad2deg(lon_target):.3f}°")
    print(f"Target Rgo at deployment: {RGO_TARGET/1e3:.1f} km")
    print(f"Bang-bang incentive weight: {BANG_BANG_WEIGHT}")
    print(f"Rgo constraint mode: {'SOFT PENALTY' if USE_RGO_PENALTY else 'HARD CONSTRAINT'}")
    print("="*70)
    
    # Initial guess: bang-bang pattern similar to target image
    # Start positive, then sharp negative, then positive, ending near zero
    sigma_init = np.zeros(N_CONTROL_POINTS)
    
    # Create a bang-bang pattern: +90° -> -90° -> +90° -> 0°
    n = N_CONTROL_POINTS
    sigma_init[:int(0.25*n)] = deg(20)      # First 25%: positive bank
    sigma_init[int(0.25*n):int(0.4*n)] = deg(85)   # Next 15%: sharp negative
    sigma_init[int(0.4*n):int(0.6*n)] = deg(-85)     # Next 20%: positive again
    sigma_init[int(0.6*n):] = deg(0)              # Last 40%: moderate negative
    
    print(f"Initial guess: bang-bang pattern with {np.count_nonzero(np.abs(sigma_init) > deg(70))} extreme points")

    # Bounds: sigma in [-90°, +90°]
    bounds = [(SIGMA_MIN, SIGMA_MAX) for _ in range(N_CONTROL_POINTS)]
    
    # Constraints
    constraints = []
    
    # Add Rgo constraint only if not using soft penalty
    if not USE_RGO_PENALTY:
        constraints.append({'type': 'eq', 'fun': rgo_constraint})
    
    # Rate and acceleration limits (optional - may prevent convergence)
    if USE_RATE_CONSTRAINTS:
        constraints.append({'type': 'ineq', 'fun': rate_constraints})
        print("Rate/acceleration constraints: ENABLED")
    else:
        print("Rate/acceleration constraints: DISABLED (for faster convergence)")
    
    # Optional: add deployment constraint
    # constraints.append({'type': 'ineq', 'fun': deployment_constraint})
    
    print("\nStarting optimization with SLSQP...\n")
    
    result = minimize(
        objective,
        sigma_init,
        method='SLSQP',
        bounds=bounds,
        constraints=constraints,
        options={
            'maxiter': 200,
            'ftol': 1e-4,
            'disp': True,
            'iprint': 2
        }
    )
    
    print("\n" + "="*70)
    print("OPTIMIZATION COMPLETE")
    print("="*70)
    print(f"Success: {result.success}")
    print(f"Message: {result.message}")
    print(f"Iterations: {result.nit}")
    print(f"Function evaluations: {result.nfev}")
    print(f"Objective (J = -h_final): {result.fun:.3f}")
    
    # Evaluate final solution
    final_state, deployed, deploy_time, hist = simulate_trajectory(result.x, return_history=True)
    r_final, lon_final, lat_final, V_final, gam_final, psi_final = final_state
    h_final = r_final - mars.radius
    
    # Compute final Rgo
    _, _, _, RC_final, RD_final, RD_go_final, Rgo_final = spherical_metrics(
        lat0, lon0, lat_final, lon_final, lat_target, lon_target, mars.radius
    )
    
    print(f"\nFinal altitude: {h_final/1e3:.2f} km")
    print(f"Final Rgo: {Rgo_final/1e3:.2f} km (target: {RGO_TARGET/1e3:.1f} km)")
    print(f"Final crossrange: {RC_final/1e3:.2f} km")
    print(f"Final downrange-to-go: {RD_final/1e3:.2f} km")
    print(f"Deployed: {deployed}")
    if deployed:
        print(f"Deployment time: {deploy_time:.1f} s")
    
    return result, hist

# ============================================================
# PLOTTING
# ============================================================
def plot_results(result, hist):
    """Plot optimization results."""
    sigma_opt = result.x
    
    # Convert history to arrays
    t = np.array(hist["t"])
    h = np.array(hist["h"]) / 1e3  # km
    V = np.array(hist["V"])
    q = np.array(hist["q"])
    mach = np.array(hist["mach"])
    gam = np.rad2deg(np.array(hist["gam"]))
    psi = np.rad2deg(np.array(hist["psi"]))
    sigma = np.rad2deg(np.array(hist["sigma"]))
    Rgo = np.array(hist["Rgo"]) / 1e3  # km
    RC = np.array(hist["RC"]) / 1e3  # km
    RD_go = np.array(hist["RD_go"]) / 1e3  # km
    
    # Actual trajectory duration
    t_final = t[-1]
    
    # Map control points to actual time
    t_control_actual = TAU_CONTROL * t_final
    
    # Calculate rates based on actual time spacing
    dt_control = t_control_actual[1] - t_control_actual[0]
    sigma_dot = np.gradient(sigma_opt, dt_control)
    sigma_ddot = np.gradient(sigma_dot, dt_control)
    
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f"NLP Optimal Trajectory (h_final = {h[-1]:.2f} km, Rgo_final = {Rgo[-1]:.2f} km)", 
                 fontsize=14, fontweight='bold')
    
    # 3x3 grid
    axes = []
    for i in range(9):
        axes.append(plt.subplot(3, 3, i+1))
    
    # Altitude
    axes[0].plot(t, h, 'b-', lw=2)
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Altitude [km]')
    axes[0].grid(True, alpha=0.3)
    
    # Velocity
    axes[1].plot(t, V, 'r-', lw=2)
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Velocity [m/s]')
    axes[1].grid(True, alpha=0.3)
    
    # Dynamic pressure
    axes[2].plot(t, q, 'g-', lw=2)
    axes[2].axhspan(300, 800, alpha=0.2, color='green', label='Deploy q-band')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('q [Pa]')
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)
    
    # Mach number
    axes[3].plot(t, mach, 'purple', lw=2)
    axes[3].axhspan(1.4, 2.2, alpha=0.2, color='purple', label='Deploy M-band')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('Mach [-]')
    axes[3].legend(fontsize=8)
    axes[3].grid(True, alpha=0.3)
    
    # Flight path angle
    axes[4].plot(t, gam, 'orange', lw=2)
    axes[4].set_xlabel('Time [s]')
    axes[4].set_ylabel('γ [deg]')
    axes[4].grid(True, alpha=0.3)
    
    # Heading angle
    axes[5].plot(t, psi, 'brown', lw=2)
    axes[5].set_xlabel('Time [s]')
    axes[5].set_ylabel('ψ [deg]')
    axes[5].grid(True, alpha=0.3)
    
    # Bank angle
    axes[6].plot(t, sigma, 'k-', lw=2, label='σ(t)')
    axes[6].plot(t_control_actual, np.rad2deg(sigma_opt), 'ro', ms=4, label='Control points')
    axes[6].axhline(90, ls='--', color='gray', alpha=0.5)
    axes[6].axhline(-90, ls='--', color='gray', alpha=0.5)
    axes[6].set_xlabel('Time [s]')
    axes[6].set_ylabel('σ [deg]')
    axes[6].legend(fontsize=8)
    axes[6].grid(True, alpha=0.3)
    
    # Bank rate
    axes[7].plot(t_control_actual, np.rad2deg(sigma_dot), 'b-', lw=2)
    axes[7].axhline(20, ls='--', color='red', alpha=0.5, label='Limit')
    axes[7].axhline(-20, ls='--', color='red', alpha=0.5)
    axes[7].set_xlabel('Time [s]')
    axes[7].set_ylabel('σ̇ [deg/s]')
    axes[7].legend(fontsize=8)
    axes[7].grid(True, alpha=0.3)
    
    # Range to go (Rgo)
    axes[8].plot(t, Rgo, 'magenta', lw=2, label='Rgo')
    axes[8].axhline(RGO_TARGET/1e3, ls='--', color='red', alpha=0.5, label=f'Target ({RGO_TARGET/1e3:.1f} km)')
    axes[8].set_xlabel('Time [s]')
    axes[8].set_ylabel('Range-to-go [km]')
    axes[8].legend(fontsize=8)
    axes[8].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Ground track plot
    fig2, ax = plt.subplots(figsize=(10, 8))
    lat_traj = np.rad2deg(np.array(hist["lat"]))
    lon_traj = np.rad2deg(np.array(hist["lon"]))
    
    # Plot trajectory
    sc = ax.scatter(lon_traj, lat_traj, c=t, cmap='viridis', s=20, label='Trajectory', zorder=3)
    
    # Mark important locations
    ax.plot(np.rad2deg(lon0), np.rad2deg(lat0), 'go', ms=12, label='Entry', zorder=5, 
            markeredgecolor='darkgreen', markeredgewidth=2)
    ax.plot(lon_traj[-1], lat_traj[-1], 'bs', ms=12, label='Deployment', zorder=5,
            markeredgecolor='darkblue', markeredgewidth=2)
    ax.plot(np.rad2deg(lon_target), np.rad2deg(lat_target), 'r*', ms=20, label='Target', zorder=5,
            markeredgecolor='darkred', markeredgewidth=1.5)
    
    # Add text annotations
    ax.text(np.rad2deg(lon0), np.rad2deg(lat0), '  Entry', fontsize=10, va='bottom')
    ax.text(lon_traj[-1], lat_traj[-1], '  Deploy', fontsize=10, va='bottom')
    ax.text(np.rad2deg(lon_target), np.rad2deg(lat_target), '  Target', fontsize=10, va='bottom')
    
    ax.set_xlabel('Longitude [deg]', fontsize=11)
    ax.set_ylabel('Latitude [deg]', fontsize=11)
    ax.set_title('Ground Track', fontsize=13, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax, label='Time [s]')
    plt.tight_layout()
    plt.show()

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    result, hist = solve_nlp()
    plot_results(result, hist)
