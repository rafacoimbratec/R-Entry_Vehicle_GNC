import numpy as np
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple, Dict, Any
import casadi as ca
from spherical_metrics import spherical_metrics

# ============================================================
# MODELS & CONSTANTS
# ============================================================
# These dataclasses define the physical properties of Mars and the entry vehicle
# They're used throughout the simulation for atmospheric and vehicle calculations

@dataclass
class Mars:
    """Mars atmospheric and gravitational model."""
    radius: float = 3396.2e3            # [m] - Mars mean radius
    mu: float = 4.282837e13             # [m^3/s^2] - Gravitational parameter (GM)
    omega: float = 7.088e-5             # [rad/s] - Planetary rotation rate
    rho0: float = 0.020                 # [kg/m^3] - Sea-level atmospheric density
    Hs: float = 11100.0                 # [m] - Atmospheric scale height (exponential decay)
    # Gas properties (CO2-dominated atmosphere)
    gamma_gas: float = 1.294            # Specific heat ratio for CO2
    R_gas: float = 188.9                # [J/(kg·K)] - Specific gas constant

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
# 3DOF EQUATIONS OF MOTION (EOM) + RK4 INTEGRATOR
# ============================================================
# These functions implement the 3-degree-of-freedom atmospheric entry dynamics
# State vector: [r, lon, lat, V, gamma, psi]
# - r: radial distance from Mars center [m]
# - lon (theta): longitude [rad]
# - lat (phi): latitude [rad]
# - V: velocity magnitude [m/s]
# - gamma: flight path angle [rad] (angle between velocity and local horizontal)
# - psi: heading angle [rad] (azimuth of velocity vector)

def eom(state, sigma):
    """3DOF equations of motion for atmospheric entry (NumPy version).
    
    Args:
        state: [r, lon, lat, V, gam, psi] state vector
        sigma: bank angle [rad] - control input
    
    Returns:
        state_dot: time derivatives of state vector
    """
    # Unpack state variables
    r, th, ph, V, gam, psi = state
    
    # Gravitational acceleration at current altitude
    g = mars.mu / r**2
    
    # Atmospheric density (exponential model)
    rho = mars.rho0 * np.exp(-(r - mars.radius) / mars.Hs)
    
    # Aerodynamic forces
    D = 0.5 * rho * V**2 / max(veh.beta, 1e-9)  # Drag acceleration [m/s^2]
    L = veh.L_over_D * D                         # Lift acceleration [m/s^2]
    
    # Horizontal velocity component
    Vh = V * np.cos(gam)

    # ===== STATE DERIVATIVES (3DOF dynamics on rotating Mars) =====
    
    # Radial position rate: component of velocity along radial direction
    r_dot = V * np.sin(gam)
    
    # Longitude rate: eastward velocity divided by local radius
    th_dot = (Vh * np.sin(psi)) / (r * max(np.cos(ph), 1e-6))
    
    # Latitude rate: northward velocity divided by radius
    ph_dot = (Vh * np.cos(psi)) / r
    
    # Velocity magnitude rate: drag, gravity, and centrifugal effects
    # Terms: -D (drag), -g*sin(gamma) (gravity), +centrifugal (Mars rotation)
    V_dot = -D - g * np.sin(gam) + mars.omega**2 * r * np.cos(ph) * \
            (np.sin(gam)*np.cos(ph) - np.cos(gam)*np.sin(ph)*np.cos(psi))
    
    # Flight path angle rate: lift (modulated by bank angle), gravity, Coriolis
    # cos(sigma): lift component normal to velocity
    # Coriolis and centrifugal terms due to Mars rotation
    gam_dot = (L*np.cos(sigma)/max(V,1e-6)) + (V/r - g/max(V,1e-6))*np.cos(gam) + \
              2*mars.omega*np.cos(ph)*np.sin(psi) + \
              (mars.omega**2*r/max(V,1e-6))*np.cos(ph)* \
              (np.cos(gam)*np.cos(ph) + np.sin(gam)*np.sin(ph)*np.cos(psi))
    
    # Heading angle rate: lateral lift (from bank angle), curvature, Coriolis
    # sin(sigma): lift component that changes heading
    psi_dot = (L*np.sin(sigma))/(max(V,1e-6)*max(np.cos(gam),1e-6)) + \
              (Vh/r)*np.sin(psi)*np.tan(ph) - \
              2*mars.omega*(np.tan(gam)*np.cos(ph)*np.cos(psi) - np.sin(ph)) + \
              (mars.omega**2*r/(max(V,1e-6)*max(np.cos(gam),1e-6)))*np.sin(ph)*np.cos(ph)*np.sin(psi)
    
    return np.array([r_dot, th_dot, ph_dot, V_dot, gam_dot, psi_dot])

def rk4_step(f, state, sigma, dt):
    """Runge-Kutta 4th order integration step.
    
    Classic RK4 provides 4th-order accuracy for numerical integration.
    Used here to generate initial guess trajectory by forward simulation.
    
    Args:
        f: dynamics function (eom)
        state: current state vector
        sigma: control input (bank angle)
        dt: time step
    
    Returns:
        state at next time step
    """
    # Evaluate dynamics at 4 points within the time step
    k1 = f(state, sigma)                  # At beginning
    k2 = f(state + 0.5*dt*k1, sigma)      # At midpoint using k1
    k3 = f(state + 0.5*dt*k2, sigma)      # At midpoint using k2
    k4 = f(state + dt*k3, sigma)          # At end using k3
    
    # Weighted combination for next state (Simpson's rule)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

# ============================================================
# NLP CONFIGURATION
# ============================================================
# This section defines the optimization problem setup

# Collocation discretization parameters
DT = 0.5           # [s] Integration timestep (not used directly, dt is optimized)
N_SEGMENTS = 100   # Number of collocation segments
                   # More segments = higher fidelity but more decision variables
                   # 100 segments → 707 decision variables total

# Entry interface conditions (atmospheric entry point)
deg = np.deg2rad   # Convenience function for degree to radian conversion
h0   = 125e3       # [m] Entry altitude (125 km)
r0   = mars.radius + h0  # [m] Entry radius from Mars center
V0   = 5000.0      # [m/s] Entry velocity
gam0 = deg(-12.0)  # [rad] Entry flight path angle (negative = descending)
psi0 = deg(-2.8758) # [rad] Entry heading angle
lat0 = deg(-21.5)  # [rad] Entry latitude
lon0 = deg(-176.40167)   # [rad] Entry longitude

# Target landing site (for reference/plotting only)
lat_target = lat0 + deg(15)   # [rad] Target latitude
lon_target = lon0 - deg(0.2)  # [rad] Target longitude

# Initial state vector: [r, lon, lat, V, gamma, psi]
state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])

# Control (bank angle) limits
# 10% margin from ±90° to avoid saturation: ±81°
SIGMA_MIN = deg(-81.0)     # [rad] Minimum bank angle (with 10% margin)
SIGMA_MAX = deg(81.0)      # [rad] Maximum bank angle (with 10% margin)
SIGMA_DOT_MAX = deg(20.0)  # [rad/s] Maximum bank angle rate
SIGMA_DOT_MIN = -deg(20.0) # [rad/s] Minimum bank angle rate
SIGMA_DOT_DOT_MAX = deg(5.0)  # [rad/s²]
SIGMA_DOT_DOT_MIN = -deg(5.0)
                           # Limits how fast vehicle can roll

# Objective function weights
# Maximize altitude while minimizing gamma^2 for control authority
W_ALTITUDE = 1.0      # Weight for altitude maximization
W_GAMMA = 95        # Weight for gamma^2 minimization (control authority)

# ============================================================
# CASADI SYMBOLIC DYNAMICS
# ============================================================
# CasADi allows us to define dynamics symbolically (not numerically)
# This enables automatic differentiation for gradient computation
# The optimizer can efficiently compute Jacobians and Hessians

def eom_casadi(state, sigma):
    """CasADi symbolic version of equations of motion.
    
    This is the SAME physics as eom(), but uses CasADi symbolic operations.
    CasADi tracks all operations to build a computation graph,
    enabling automatic differentiation for the optimizer.
    
    Key difference from NumPy version:
    - ca.exp() instead of np.exp()
    - ca.fmax() instead of max() (protects against division by zero)
    - All operations are symbolic, not numerical
    
    Args:
        state: CasADi symbolic state vector [r, lon, lat, V, gam, psi]
        sigma: CasADi symbolic bank angle
    
    Returns:
        CasADi symbolic state derivative vector
    """
    # Unpack state (CasADi indexing)
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

def compute_spherical_metrics_casadi(lat, lon, lat_t, lon_t, R):
    """CasADi symbolic version of spherical distance calculation."""
    # Great circle distance (Rgo)
    dlat = lat - lat_t
    dlon = lon - lon_t
    a = ca.sin(dlat/2)**2 + ca.cos(lat) * ca.cos(lat_t) * ca.sin(dlon/2)**2
    c = 2 * ca.atan2(ca.sqrt(a), ca.sqrt(1-a))
    Rgo = R * c
    return Rgo

# ============================================================
# DIRECT COLLOCATION OPTIMIZATION
# ============================================================
def solve_collocation(lat_target_in=None, lon_target_in=None):
    """
    Solve trajectory optimization using direct collocation with CasADi.
    Uses Hermite-Simpson collocation for better accuracy.
    
    Args:
        lat_target_in: Target latitude [rad] (optional, uses default if None)
        lon_target_in: Target longitude [rad] (optional, uses default if None)
    """
    # Use provided target or default values
    lat_target_use = lat_target_in if lat_target_in is not None else lat_target
    lon_target_use = lon_target_in if lon_target_in is not None else lon_target
    
    print("\n" + "="*70)
    print("DIRECT COLLOCATION OPTIMIZATION WITH CASADI")
    print("="*70)
    print(f"Collocation segments: {N_SEGMENTS}")
    print(f"Integration timestep: {DT} s")
    print(f"Objective: Maximize altitude while minimizing gamma^2")
    print(f"Weights: W_altitude={W_ALTITUDE}, W_gamma={W_GAMMA}")
    print("="*70)
    
    # Create CasADi optimization problem (Opti stack)
    # Opti provides a high-level interface to define NLP problems
    opti = ca.Opti()
    
    # ===== DECISION VARIABLES =====
    # Unlike shooting methods, we optimize BOTH states AND controls!
    # Total variables: 6*(N+1) states + N controls + 1 time = 707 vars
    
    # State trajectory: r, lon, lat, V, gam, psi at each node
    # Shape: (6, 101) → 606 state variables
    X = opti.variable(6, N_SEGMENTS + 1)  
    
    # Control trajectory: bank angle at each segment
    # Shape: (1, 100) → 100 control variables
    U = opti.variable(1, N_SEGMENTS)      
    
    # Time step: FREE variable (optimizer chooses optimal dt)
    # This allows variable-duration trajectory
    dt = opti.variable()                  
    
    # Extract state components for readability in constraints
    # Each is a vector of length (N_SEGMENTS+1)
    r = X[0, :]      # Radial position at each node
    lon = X[1, :]    # Longitude at each node
    lat = X[2, :]    # Latitude at each node
    V = X[3, :]      # Velocity at each node
    gam = X[4, :]    # Flight path angle at each node
    psi = X[5, :]    # Heading angle at each node
    
    # Control at each segment (length N_SEGMENTS)
    sigma = U[0, :]  # Bank angle profile
    
    # ===== OBJECTIVE FUNCTION =====
    # Maximize altitude while minimizing gamma^2 for control authority
    # CasADi minimizes, so we use negative altitude
    h_final = r[-1] - mars.radius  # Altitude at final node [m]
    gam_final = gam[-1]             # Final flight path angle [rad]
    
    # Weighted combination: maximize altitude, minimize gamma^2
    # -h_final: maximize altitude (negative for minimization)
    # +W_GAMMA * gam_final^2: penalize steep flight path angles
    objective = -W_ALTITUDE * h_final + W_GAMMA * gam_final**2
    opti.minimize(objective)
    
    # ===== BOUNDARY CONDITIONS =====
    # Fix initial state to entry interface conditions
    # This is an equality constraint: X[:, 0] = state0
    opti.subject_to(X[:, 0] == state0)
    
    # ===== DYNAMICS CONSTRAINTS (COLLOCATION) =====
    # This is the KEY difference from shooting methods!
    # We don't simulate forward; instead we enforce dynamics as constraints
    # Loop through each segment and apply Hermite-Simpson rule
    for k in range(N_SEGMENTS):
        # Get state and control for this segment
        x_k = X[:, k]      # State at beginning of segment k
        x_kp1 = X[:, k+1]  # State at end of segment k (beginning of k+1)
        u_k = U[:, k]      # Control during segment k (held constant)
        
        # Compute midpoint state by linear interpolation
        # Hermite-Simpson assumes cubic interpolation, starts with linear guess
        x_mid = 0.5 * (x_k + x_kp1)
        
        # Evaluate dynamics (state derivatives) at 3 points:
        f_k = eom_casadi(x_k, u_k)      # Derivative at segment start
        f_kp1 = eom_casadi(x_kp1, u_k)  # Derivative at segment end
        f_mid = eom_casadi(x_mid, u_k)  # Derivative at midpoint
        
        # HERMITE-SIMPSON COLLOCATION CONSTRAINT:
        # Enforce that the state transition satisfies Simpson's integration rule
        # This is equivalent to: ∫[t_k to t_k+1] f(x(t), u) dt = (dt/6)(f_k + 4*f_mid + f_k+1)
        # The optimizer must find X and U such that this holds for ALL segments
        opti.subject_to(x_kp1 == x_k + (dt / 6) * (f_k + 4*f_mid + f_kp1))
    
    # ===== PATH CONSTRAINTS =====
    # These constraints must hold at EVERY node (unlike boundary conditions)
    
    # Control bounds: limit bank angle to physically realizable range
    # Applied to all 100 control points simultaneously
    opti.subject_to(opti.bounded(SIGMA_MIN, sigma, SIGMA_MAX))
    
    # State bounds to prevent numerical issues and ensure physical validity
    opti.subject_to(r >= mars.radius)  # Don't go underground!
    opti.subject_to(V >= 10.0)         # Keep velocity positive
    opti.subject_to(opti.bounded(deg(-89), gam, deg(89)))  # Avoid singularities
    
    # Control rate constraint: limit how fast bank angle can change
    # This enforces actuator rate limits (vehicle can't roll infinitely fast)
    for k in range(N_SEGMENTS - 1):
        # Approximate sigma_dot using finite difference
        sigma_dot = (sigma[k+1] - sigma[k]) / dt
        # Constrain rate to ±20 deg/s
        opti.subject_to(opti.bounded(-SIGMA_DOT_MAX, sigma_dot, SIGMA_DOT_MAX))
    
    # Control acceleration constraint: limit how fast bank rate can change
    # This enforces actuator acceleration limits (smooth control changes)
    for k in range(N_SEGMENTS - 2):
        # Approximate sigma_dot_dot using second-order finite difference
        sigma_dot_k = (sigma[k+1] - sigma[k]) / dt
        sigma_dot_kp1 = (sigma[k+2] - sigma[k+1]) / dt
        sigma_dot_dot = (sigma_dot_kp1 - sigma_dot_k) / dt
        # Constrain acceleration to ±5 deg/s²
        opti.subject_to(opti.bounded(-SIGMA_DOT_DOT_MAX, sigma_dot_dot, SIGMA_DOT_DOT_MAX))
    
    # Bound on time step variable
    # dt must be positive and reasonable (0.1 to 5.0 seconds per segment)
    opti.subject_to(opti.bounded(0.1, dt, 5.0))
    
    # ===== TERMINAL CONSTRAINTS =====
    # Enforce deployment within safe envelope constraints
    
    # Define deployment constraints
    MACH_MIN = 1.4    # Minimum Mach number for deployment
    MACH_MAX = 2.2    # Maximum Mach number for deployment
    Q_MIN = 300.0     # [Pa] Minimum dynamic pressure
    Q_MAX = 800.0     # [Pa] Maximum dynamic pressure
    H_MIN = 6.0e3     # [m] Minimum altitude for deployment (6 km)
    
    # Extract final state
    r_f = r[-1]    # Final radius
    V_f = V[-1]    # Final velocity
    h_f = r_f - mars.radius  # Final altitude
    
    # Compute atmospheric properties at deployment
    rho_f = mars.rho0 * ca.exp(-h_f / mars.Hs)  # Density
    q_f = 0.5 * rho_f * V_f**2                   # Dynamic pressure [Pa]
    
    # Compute Mach number at deployment
    # Need temperature first (polynomial fit to Mars atmosphere)
    h_km_f = h_f / 1e3
    T_f = 1.4e-13 * h_km_f**3 - 8.85e-9 * h_km_f**2 - 1.245e-3 * h_km_f + 205.36
    a_sound_f = ca.sqrt(mars.gamma_gas * mars.R_gas * ca.fmax(T_f, 1.0))  # Speed of sound
    mach_f = V_f / ca.fmax(a_sound_f, 1e-6)  # Mach number
    
    # DEPLOYMENT WINDOW CONSTRAINTS:
    # Vehicle must deploy within safe Mach, q, and altitude envelope
    opti.subject_to(opti.bounded(MACH_MIN, mach_f, MACH_MAX))  # 1.0 ≤ M ≤ 4.0
    opti.subject_to(opti.bounded(Q_MIN, q_f, Q_MAX))           # 300 ≤ q ≤ 800 Pa
    opti.subject_to(h_f >= H_MIN)                              # h ≥ 6 km
    
    # ===== GENERATE INITIAL GUESS BY FORWARD SIMULATION =====
    # Good initial guess is CRITICAL for NLP convergence
    # Bad guess → NaN errors, failed optimization
    # We generate a feasible (but suboptimal) trajectory using RK4 simulation
    
    print("Generating initial guess via forward simulation...")
    state_sim = state0.copy()            # Start at entry interface
    X_guess = np.zeros((6, N_SEGMENTS + 1))  # Allocate guess array
    X_guess[:, 0] = state0                    # Set initial condition
    
    # Use bang-bang structure for initial control guess: 0 → 81 → -81 → 0
    # This provides a richer initial guess than constant bank angle
    dt_guess = 1.5         # Time step for simulation
    
    # Define bang-bang switching points (as fractions of total segments)
    switch_1 = int(0.25 * N_SEGMENTS)  # First switch at 25%
    switch_2 = int(0.50 * N_SEGMENTS)  # Second switch at 50%
    switch_3 = int(0.75 * N_SEGMENTS)  # Third switch at 75%
    
    # Simulate forward using RK4 to fill state guess
    for k in range(N_SEGMENTS):
        # Determine bank angle based on bang-bang structure
        if k < switch_1:
            sigma_k = deg(0)      # Phase 1: 0 degrees (straight)
        elif k < switch_2:
            sigma_k = deg(81)     # Phase 2: +81 degrees (right roll, 10% margin)
        elif k < switch_3:
            sigma_k = deg(-81)    # Phase 3: -81 degrees (left roll, 10% margin)
        else:
            sigma_k = deg(0)      # Phase 4: 0 degrees (straight)
        
        # Integrate one step forward
        state_sim = rk4_step(eom, state_sim, sigma_k, dt_guess)
        X_guess[:, k+1] = state_sim
        
        # Check if we hit the ground (safety check)
        h_sim = state_sim[0] - mars.radius
        if h_sim < 0:
            # If ground impact, hold final state for remaining nodes
            X_guess[:, k+1:] = state_sim[:, np.newaxis]
            break
    
    print(f"Initial guess trajectory: h_final = {(X_guess[0,-1] - mars.radius)/1e3:.1f} km")
    
    # Set initial guess for state trajectory
    # This tells IPOPT where to start searching
    opti.set_initial(X, X_guess)
    
    # Initial guess for control: bang-bang structure
    U_guess = np.zeros((1, N_SEGMENTS))
    for k in range(N_SEGMENTS):
        if k < switch_1:
            U_guess[0, k] = deg(0)
        elif k < switch_2:
            U_guess[0, k] = deg(81)
        elif k < switch_3:
            U_guess[0, k] = deg(-81)
        else:
            U_guess[0, k] = deg(0)
    opti.set_initial(U, U_guess)
    
    # Initial guess for time step
    opti.set_initial(dt, 1.5)  # Start with 1.5 s per segment
    
    # ===== SOLVER CONFIGURATION =====
    # IPOPT = Interior Point OPTimizer (industry-standard NLP solver)
    # Uses interior-point method with line search and barrier functions
    
    opts = {
        # Verbosity level (0=silent, 5=detailed iteration log)
        'ipopt.print_level': 5,
        
        # Maximum iterations before giving up
        'ipopt.max_iter': 3000,
        
        # Convergence tolerance (tight = 1e-8, loose = 1e-3)
        'ipopt.tol': 1e-6,
        
        # Acceptable tolerance (if optimal not reached, accept this)
        'ipopt.acceptable_tol': 1e-4,
        
        # Barrier parameter update strategy (adaptive is robust)
        'ipopt.mu_strategy': 'adaptive',
        
        # How to scale the NLP (gradient-based helps with poorly scaled problems)
        'ipopt.nlp_scaling_method': 'gradient-based',
        
        # Print timing information
        'print_time': True
    }
    
    # Tell CasADi to use IPOPT solver with these options
    opti.solver('ipopt', opts)
    
    # ===== SOLVE THE OPTIMIZATION PROBLEM =====
    # This is where the magic happens!
    # IPOPT will iteratively adjust X, U, and dt to:
    # 1. Minimize the objective (-h_final)
    # 2. Satisfy all constraints (dynamics, bounds, terminal conditions)
    # 3. Find a local optimum using interior-point method
    
    print("\nStarting IPOPT solver...\n")
    start_time = time.perf_counter()
    
    try:
        # Attempt to solve the NLP
        # If successful, sol contains optimal X, U, dt
        sol = opti.solve()
        success = True
        print("\n" + "="*70)
        print("OPTIMIZATION SUCCESSFUL")
        print("="*70)
    except RuntimeError as e:
        # If solver fails to converge, retrieve best attempt
        # Debug solution may violate some constraints but still useful
        print("\n" + "="*70)
        print("OPTIMIZATION FAILED - Retrieving debug solution")
        print("="*70)
        sol = opti.debug  # Get last iterate before failure
        success = False
    
    elapsed = time.perf_counter() - start_time
    
    # ===== EXTRACT SOLUTION =====
    # Convert CasADi symbolic solution to NumPy arrays for plotting
    
    # Get optimal state trajectory (6 x 101 array)
    X_sol = sol.value(X)
    
    # Get optimal control trajectory (1 x 100 array)
    U_sol = sol.value(U)
    
    # Get optimal time step (scalar)
    dt_sol = sol.value(dt)
    
    # Compute time vector: uniformly spaced with optimal dt
    # Total trajectory time = N_SEGMENTS * dt_sol
    t_sol = np.arange(N_SEGMENTS + 1) * dt_sol
    
    # Flatten control array if needed (CasADi may return 2D)
    if U_sol.ndim == 2:
        U_sol_flat = U_sol.flatten()  # (1, 100) → (100,)
    else:
        U_sol_flat = U_sol  # Already 1D
    
    # ===== BUILD HISTORY DICTIONARY =====
    # Package solution into convenient format for plotting
    
    hist = {
        "t": t_sol,  # Time vector [s]
        "h": (X_sol[0, :] - mars.radius) / 1e3,  # Altitude [km]
        "V": X_sol[3, :],    # Velocity [m/s]
        "gam": X_sol[4, :],  # Flight path angle [rad]
        "psi": X_sol[5, :],  # Heading angle [rad]
        "lat": X_sol[2, :],  # Latitude [rad]
        "lon": X_sol[1, :],  # Longitude [rad]
        # Control has N points, states have N+1
        # Repeat last control value for plotting alignment
        "sigma": np.append(U_sol_flat, U_sol_flat[-1]),  # Bank angle [rad]
    }
    
    # Compute derived quantities (not directly optimized)
    # These are computed from the state trajectory
    hist["rho"] = []     # Atmospheric density [kg/m³]
    hist["q"] = []       # Dynamic pressure [Pa]
    hist["mach"] = []    # Mach number [-]
    hist["Rgo"] = []     # Range-to-go [km]
    hist["RC"] = []      # Crossrange [km]
    hist["RD"] = []      # Downrange [km]
    hist["RD_go"] = []   # Downrange-to-go [km]
    
    # Loop through each node and compute derived quantities
    for i in range(N_SEGMENTS + 1):
        # Extract state at node i
        r_i = X_sol[0, i]
        V_i = X_sol[3, i]
        lat_i = X_sol[2, i]
        lon_i = X_sol[1, i]
        h_i = r_i - mars.radius  # Altitude
        
        # Atmospheric density (exponential model)
        rho_i = mars.rho0 * np.exp(-h_i / mars.Hs)
        
        # Dynamic pressure: q = 0.5 * ρ * V²
        q_i = 0.5 * rho_i * V_i**2
        
        # Mach number: M = V / a (speed of sound)
        h_km_i = h_i / 1e3
        T_i = 1.4e-13 * h_km_i**3 - 8.85e-9 * h_km_i**2 - 1.245e-3 * h_km_i + 205.36  # Temperature
        a_sound_i = np.sqrt(mars.gamma_gas * mars.R_gas * max(T_i, 1.0))  # Speed of sound
        mach_i = V_i / max(a_sound_i, 1e-6)
        
        # Compute spherical geometry metrics
        # Returns: azimuth angles and distances (crossrange, downrange, range-to-go)
        # Compute spherical geometry metrics
        # Returns: azimuth angles and distances (crossrange, downrange, range-to-go)
        _, _, _, RC_i, RD_i, RD_go_i, Rgo_i = spherical_metrics(
            lat0, lon0, lat_i, lon_i, lat_target_use, lon_target_use, mars.radius
        )
        
        # Append to lists (will convert to arrays after loop)
        hist["rho"].append(rho_i)
        hist["q"].append(q_i)
        hist["mach"].append(mach_i)
        hist["Rgo"].append(Rgo_i / 1e3)    # Convert to km
        hist["RC"].append(RC_i / 1e3)      # Convert to km
        hist["RD"].append(RD_i / 1e3)      # Convert to km
        hist["RD_go"].append(RD_go_i / 1e3)  # Convert to km
    
    # Convert lists to numpy arrays for easier manipulation
    for key in ["rho", "q", "mach", "Rgo", "RC", "RD", "RD_go"]:
        hist[key] = np.array(hist[key])
    
    # ===== PRINT SOLUTION SUMMARY =====
    print(f"\nSolution time: {elapsed:.1f} s")  # How long IPOPT took
    print(f"Final altitude: {hist['h'][-1]:.2f} km")  # Deployment altitude (objective)
    print(f"Final gamma: {np.rad2deg(hist['gam'][-1]):.2f} deg")  # Flight path angle
    print(f"Final Rgo: {hist['Rgo'][-1]:.2f} km")  # Range to target
    print(f"Final crossrange: {hist['RC'][-1]:.2f} km")  # Lateral displacement
    print(f"Final Mach: {hist['mach'][-1]:.2f}")  # Mach number at deployment
    print(f"Final q: {hist['q'][-1]:.1f} Pa")  # Dynamic pressure at deployment
    print(f"Trajectory duration: {t_sol[-1]:.1f} s")  # Total entry flight time
    print(f"Time step: {dt_sol:.3f} s")  # Optimal time step found by solver
    
    return sol, hist, success

# ============================================================
# PLOTTING
# ============================================================
def plot_results(sol, hist):
    """Plot collocation optimization results.
    
    Creates two figures:
    1. 3x3 grid of state and control time histories
    2. Ground track (lat/lon trajectory)
    
    Args:
        sol: CasADi solution object (not used here)
        hist: trajectory history dictionary
    """
    
    # ===== EXTRACT DATA FOR PLOTTING =====
    # Convert history to convenient arrays
    t = hist["t"]      # Time vector [s]
    h = hist["h"]      # Altitude [km] - already converted
    V = hist["V"]      # Velocity [m/s]
    q = hist["q"]      # Dynamic pressure [Pa]
    mach = hist["mach"]  # Mach number
    gam = np.rad2deg(hist["gam"])   # Flight path angle [deg]
    psi = np.rad2deg(hist["psi"])   # Heading angle [deg]
    sigma = np.rad2deg(hist["sigma"])  # Bank angle [deg]
    Rgo = hist["Rgo"]  # Range-to-go [km]
    RC = hist["RC"]    # Crossrange [km]
    RD_go = hist["RD_go"]  # Downrange-to-go [km]
    
    # Calculate control rates using numerical differentiation
    # These show how aggressively the bank angle changes
    dt_arr = np.diff(t)  # Time differences between nodes
    sigma_rad = hist["sigma"]  # Bank angle in radians
    sigma_dot = np.gradient(sigma_rad, t)  # Bank rate [rad/s]
    sigma_ddot = np.gradient(sigma_dot, t)  # Bank acceleration [rad/s²]
    
    # ===== CREATE FIGURE 1: TIME HISTORIES =====
    # 3x3 grid showing evolution of all important quantities
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f"Collocation Optimal Trajectory (h_final = {h[-1]:.2f} km, Rgo_final = {Rgo[-1]:.2f} km)", 
                 fontsize=14, fontweight='bold')
    
    # Create subplot grid
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
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('q [Pa]')
    axes[2].grid(True, alpha=0.3)
    
    # Mach number
    axes[3].plot(t, mach, 'purple', lw=2)
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('Mach [-]')
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
    axes[6].plot(t, sigma, 'k-', lw=2)
    axes[6].axhline(81, ls='--', color='red', alpha=0.5, label='Limit (±81°)')
    axes[6].axhline(-81, ls='--', color='red', alpha=0.5)
    axes[6].axhline(90, ls=':', color='gray', alpha=0.3, label='Physical limit (±90°)')
    axes[6].axhline(-90, ls=':', color='gray', alpha=0.3)
    axes[6].set_xlabel('Time [s]')
    axes[6].set_ylabel('σ [deg]')
    axes[6].legend(fontsize=8)
    axes[6].grid(True, alpha=0.3)
    
    # Bank rate
    axes[7].plot(t, np.rad2deg(sigma_dot), 'b-', lw=2)
    axes[7].axhline(20, ls='--', color='red', alpha=0.5, label='Limit')
    axes[7].axhline(-20, ls='--', color='red', alpha=0.5)
    axes[7].set_xlabel('Time [s]')
    axes[7].set_ylabel('σ̇ [deg/s]')
    axes[7].legend(fontsize=8)
    axes[7].grid(True, alpha=0.3)
    
    # Bank acceleration
    axes[8].plot(t, np.rad2deg(sigma_ddot), 'r-', lw=2)
    axes[8].axhline(5, ls='--', color='red', alpha=0.5, label='Limit')
    axes[8].axhline(-5, ls='--', color='red', alpha=0.5)
    axes[8].set_xlabel('Time [s]')
    axes[8].set_ylabel('σ̈ [deg/s²]')
    axes[8].legend(fontsize=8)
    axes[8].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # ===== ALTITUDE VS VELOCITY ENVELOPE PLOT =====
    # Create deployment envelope showing safe deployment region
    fig3, ax3 = plt.subplots(figsize=(10, 8))
    
    # Define deployment constraints
    MACH_MIN = 1.4  # Minimum Mach number for deployment
    MACH_MAX = 2.2    # Maximum Mach number for deployment
    Q_MIN = 300.0     # [Pa] Minimum dynamic pressure
    Q_MAX = 800.0     # [Pa] Maximum dynamic pressure
    H_MIN = 6.0       # [km] Minimum altitude for deployment
    
    # Generate velocity range for envelope calculation
    V_range = np.linspace(300, 500, 500)  # [m/s]
    
    # For each velocity, compute altitude constraints from Mach and q limits
    h_mach_min = []
    h_mach_max = []
    h_q_min = []
    h_q_max = []
    
    for V_i in V_range:
        # Use iterative approach to find altitude for each constraint
        # Start with a reasonable altitude range
        h_test = np.linspace(0, 50e3, 1000)  # [m]
        
        # Compute Mach number at each test altitude
        h_km_test = h_test / 1e3
        T_test = 1.4e-13 * h_km_test**3 - 8.85e-9 * h_km_test**2 - 1.245e-3 * h_km_test + 205.36
        a_sound_test = np.sqrt(mars.gamma_gas * mars.R_gas * np.maximum(T_test, 1.0))
        mach_test = V_i / a_sound_test
        
        # Find altitude where Mach = MACH_MIN and MACH_MAX
        idx_mach_min = np.argmin(np.abs(mach_test - MACH_MIN))
        idx_mach_max = np.argmin(np.abs(mach_test - MACH_MAX))
        h_mach_min.append(h_test[idx_mach_min] / 1e3)  # Convert to km
        h_mach_max.append(h_test[idx_mach_max] / 1e3)
        
        # Compute dynamic pressure at each test altitude
        rho_test = mars.rho0 * np.exp(-h_test / mars.Hs)
        q_test = 0.5 * rho_test * V_i**2
        
        # Find altitude where q = Q_MIN and Q_MAX
        idx_q_min = np.argmin(np.abs(q_test - Q_MIN))
        idx_q_max = np.argmin(np.abs(q_test - Q_MAX))
        h_q_min.append(h_test[idx_q_min] / 1e3)  # Convert to km
        h_q_max.append(h_test[idx_q_max] / 1e3)
    
    # Convert to arrays
    h_mach_min = np.array(h_mach_min)
    h_mach_max = np.array(h_mach_max)
    h_q_min = np.array(h_q_min)
    h_q_max = np.array(h_q_max)
    
    # Compute envelope: intersection of all constraints
    #NOTE: As altitude increases, both Mach and q DECREASE
    # For Mach constraints: MACH_MIN ≤ M ≤ MACH_MAX
    #   - h_mach_max gives altitude where M = MACH_MAX (lower altitude, higher Mach)
    #   - h_mach_min gives altitude where M = MACH_MIN (higher altitude, lower Mach)
    # For q constraints: Q_MIN ≤ q ≤ Q_MAX
    #   - h_q_max gives altitude where q = Q_MAX (lower altitude, higher q)
    #   - h_q_min gives altitude where q = Q_MIN (higher altitude, lower q)
    
    # Lower bound: max of (h at M=MACH_MAX, h at q=Q_MAX, H_MIN)
    h_lower = np.maximum.reduce([h_mach_max, h_q_max, np.full_like(V_range, H_MIN)])
    # Upper bound: min of (h at M=MACH_MIN, h at q=Q_MIN)
    h_upper = np.minimum.reduce([h_mach_min, h_q_min])
    
    # Plot the envelope
    ax3.fill_between(V_range, h_lower, h_upper, alpha=0.3, color='green', label='Safe Deployment Envelope')
    ax3.plot(V_range, h_lower, 'g-', lw=2, label='Envelope Boundary')
    ax3.plot(V_range, h_upper, 'g-', lw=2)
    
    # Plot individual constraint boundaries (for reference)
    ax3.plot(V_range, h_mach_min, 'b--', lw=1, alpha=0.5, label=f'Mach = {MACH_MIN}')
    ax3.plot(V_range, h_mach_max, 'b--', lw=1, alpha=0.5, label=f'Mach = {MACH_MAX}')
    ax3.plot(V_range, h_q_min, 'r--', lw=1, alpha=0.5, label=f'q = {Q_MIN} Pa')
    ax3.plot(V_range, h_q_max, 'r--', lw=1, alpha=0.5, label=f'q = {Q_MAX} Pa')
    ax3.axhline(H_MIN, color='purple', ls='--', lw=1, alpha=0.5, label=f'h_min = {H_MIN} km')
    
    # Plot the trajectory
    ax3.plot(V, h, 'k-', lw=2.5, label='Trajectory', zorder=5)
    
    # Mark the final (deployment) point
    ax3.plot(V[-1], h[-1], 'r*', ms=20, label='Deployment Point', zorder=10,
            markeredgecolor='darkred', markeredgewidth=1.5)
    
    ax3.set_xlabel('Velocity [m/s]', fontsize=12)
    ax3.set_ylabel('Altitude [km]', fontsize=12)
    ax3.set_title('Altitude-Velocity Envelope with Deployment Constraints', fontsize=13, fontweight='bold')
    ax3.legend(loc='best', fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim([V_range[0], V_range[-1]])
    ax3.set_ylim([0, max(h_upper.max(), h.max()) * 1.1])
    plt.tight_layout()
    plt.show()
    
    # Ground track plot
    fig2, ax = plt.subplots(figsize=(10, 8))
    lat_traj = np.rad2deg(hist["lat"])
    lon_traj = np.rad2deg(hist["lon"])
    
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
    ax.set_title('Ground Track (Collocation Solution)', fontsize=13, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax, label='Time [s]')
    plt.tight_layout()
    plt.show()

# ============================================================
# MAIN EXECUTION
# ============================================================
if __name__ == "__main__":
    # Run the collocation optimizer
    # This will:
    # 1. Set up the NLP with 707 decision variables
    # 2. Generate initial guess via forward simulation
    # 3. Call IPOPT to solve the optimization problem
    # 4. Return optimal trajectory and success flag
    sol, hist, success = solve_collocation()
    
    # Visualize the results
    # Creates time history plots and ground track
    plot_results(sol, hist)
