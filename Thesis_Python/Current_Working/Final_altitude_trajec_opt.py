import numpy as np
import time, math
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple, Dict, Any
from scipy.optimize import minimize

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
        """
        Atmospheric temperature model (simplified; altitude-dependent).

        Args:
            h: Altitude above mean radius [m]

        Returns:
            Temperature [K]
        """
        # Old Lee et al. (2024) model provided by user
        h_km = h / 1e3
        T = 1.4e-13 * h_km**3 - 8.85e-9 * h_km**2 - 1.245e-3 * h_km + 205.36
        return T

    def sound_speed(self, h: float) -> float:
        """Local speed of sound [m/s] using temperature(h)."""
        T = max(self.temperature(h), 1.0)  # guard against nonphysical lows
        return math.sqrt(self.gamma_gas * self.R_gas * T)


@dataclass
class Vehicle:
    beta: float = 135.0                 # ballistic coefficient [kg/m^2]
    L_over_D: float = 0.24              # lift-to-drag

mars = Mars()
veh  = Vehicle()

# ============================================================
# USER 3DOF EOM + RK4
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
# SIM CONFIG & LIMITS
# ============================================================
DT = 0.1
T_END = 1200.0

# Initial conditions (your values)
deg = np.deg2rad
h0   = 120e3
r0   = mars.radius + h0
V0   = 4700.0
gam0 = deg(-12.0)
psi0 = deg(-2.8758)
lat0 = deg(-21.3)
lon0 = deg(-176.40167)
state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])

# Bank limits (magnitudes)
SIGMA_MIN_DEG = -90.0
SIGMA_MAX_DEG =  90.0
SIGMA_DOT_MAX_DEG_S  = 20.0      # |sigma_dot| ≤ 20 deg/s
SIGMA_DDOT_MAX_DEG_S2 = 5.0      # |sigma_ddot| ≤ 5 deg/s^2

# Convert to radians for internal limiter
SIGMA_MIN = np.deg2rad(SIGMA_MIN_DEG)
SIGMA_MAX = np.deg2rad(SIGMA_MAX_DEG)
SIGMA_DOT_MAX = np.deg2rad(SIGMA_DOT_MAX_DEG_S)
SIGMA_DDOT_MAX = np.deg2rad(SIGMA_DDOT_MAX_DEG_S2)

# Reversal windows (softly encouraged by penalties)
T1_MIN, T1_MAX = 5.0, 400.0
T2_MIN, T2_MAX = 30.0, 800.0
T3_MIN, T3_MAX = 60.0, 1100.0

# Initial guess [t1, t2, t3, s1, s2, s3, s4] (deg)
x0 = np.array([90, 120, 140, 0, -80.0, 85, 0])

# ============================================================
# BANK COMMAND PROFILE + SLEW-LIMITED TRACKER
# ============================================================
def build_times_sorted(t1, t2, t3):
    ts = np.clip(np.sort(np.array([t1, t2, t3], dtype=float)), 0.0, T_END)
    return ts[0], ts[1], ts[2]

def sigma_cmd_of_time(t, t1, t2, t3, s1, s2, s3, s4):
    """Piecewise-constant command in radians (clamped to ±90°)."""
    if t < t1:   s = s1
    elif t < t2: s = s2
    elif t < t3: s = s3
    else:        s = s4
    return np.deg2rad(np.clip(s, SIGMA_MIN_DEG, SIGMA_MAX_DEG))

class SigmaLimiterState:
    def __init__(self, sigma0=0.0, sigmadot0=0.0):
        self.sigma = float(np.clip(sigma0, SIGMA_MIN, SIGMA_MAX))
        self.sigmadot = float(np.clip(sigmadot0, -SIGMA_DOT_MAX, SIGMA_DOT_MAX))

def advance_sigma_limiter(state: SigmaLimiterState, sigma_cmd: float, dt: float) -> Tuple[float,float,float]:
    """
    Second-order rate/accel limiter that tracks sigma_cmd.
    Enforces: |sigma| ≤ 90°, |sigma_dot| ≤ 20°/s, |sigma_ddot| ≤ 5°/s² (all in radians).
    Returns (sigma, sigma_dot, sigma_ddot).
    """
    # Desired accel towards command (critically damped like)
    k_vel = 4.0    # tracking gain on velocity error
    k_pos = 8.0    # tracking gain on position error
    sig_err = sigma_cmd - state.sigma
    # Proportional-derivative desired accel
    sigma_ddot_des = k_pos*sig_err - k_vel*state.sigmadot

    # Saturate accel
    sigma_ddot = float(np.clip(sigma_ddot_des, -SIGMA_DDOT_MAX, SIGMA_DDOT_MAX))

    # Integrate velocity with accel limit
    sigmadot_new = state.sigmadot + sigma_ddot*dt
    sigmadot_new = float(np.clip(sigmadot_new, -SIGMA_DOT_MAX, SIGMA_DOT_MAX))

    # Integrate position and clip to bounds
    sigma_new = state.sigma + sigmadot_new*dt

    # If we hit position bounds, prevent integrating further out
    if sigma_new > SIGMA_MAX:
        sigma_new = SIGMA_MAX
        if sigmadot_new > 0.0: sigmadot_new = 0.0
    elif sigma_new < SIGMA_MIN:
        sigma_new = SIGMA_MIN
        if sigmadot_new < 0.0: sigmadot_new = 0.0

    # Update internal state
    state.sigma = sigma_new
    state.sigmadot = sigmadot_new
    return sigma_new, sigmadot_new, sigma_ddot

# ============================================================
# TRAJECTORY + DEPLOYMENT TRIGGER
# ============================================================
def simulate_profile(params, return_history=False):
    """
    params: [t1,t2,t3,s1,s2,s3,s4] (times in s, angles in deg)
    returns: final_alt (m) at stop (deploy/impact/timeout), and optionally history dict
    """
    t1, t2, t3, s1, s2, s3, s4 = params
    t1, t2, t3 = build_times_sorted(t1, t2, t3)

    t = 0.0
    state = state0.copy()
    # Initialize sigma limiter at the initial commanded bank (smooth start)
    sigma0_cmd = sigma_cmd_of_time(0.0, t1, t2, t3, s1, s2, s3, s4)
    sig_state = SigmaLimiterState(sigma0_cmd, 0.0)

    deployed = False
    deploy_time = None

    if return_history:
        hist = {"t":[], "h":[], "V":[], "gam":[], "psi":[], "th":[], "ph":[],
                "rho":[], "q":[], "mach":[],
                "sigma_cmd":[], "sigma":[], "sigma_dot":[], "sigma_ddot":[],
                "deploy_flag":[]}

    while t < T_END:
        r, th, ph, V, gam, psi = state
        h = r - mars.radius
        rho = mars.rho0 * np.exp(-(r - mars.radius) / mars.Hs)
        q   = 0.5 * rho * V**2
        a_sound = mars.sound_speed(h)
        mach = V / max(a_sound, 1e-6)

        # Deployment trigger
        deploy_cond = (1.4 <= mach <= 2.2) and (300.0 <= q <= 800.0)

        if h <= 0.0 or deploy_cond:
            deployed = True
            deploy_time = t
            break

        # bank command & slew-limited application
        sigma_c = sigma_cmd_of_time(t, t1, t2, t3, s1, s2, s3, s4)
        sigma, sigma_dot, sigma_ddot = advance_sigma_limiter(sig_state, sigma_c, DT)

        # record history
        if return_history:
            hist["t"].append(t)
            hist["h"].append(h)
            hist["V"].append(V)
            hist["gam"].append(gam)
            hist["psi"].append(psi)
            hist["th"].append(th)
            hist["ph"].append(ph)
            hist["rho"].append(rho)
            hist["q"].append(q)
            hist["mach"].append(mach)
            hist["sigma_cmd"].append(sigma_c)
            hist["sigma"].append(sigma)
            hist["sigma_dot"].append(sigma_dot)
            hist["sigma_ddot"].append(sigma_ddot)
            hist["deploy_flag"].append(1 if deploy_cond else 0)

        # integrate dynamics one step using APPLIED sigma
        state = rk4_step(eom, state, sigma, DT)
        t += DT

        # impact safeguard
        if state[0] <= mars.radius:
            break

    final_alt = state[0] - mars.radius
    out = (final_alt, hist, deployed, deploy_time) if return_history else final_alt
    return out

# ============================================================
# OBJECTIVE & OPTIMIZERS
# ============================================================
class EvalCounter:
    def __init__(self): self.count = 0
    def inc(self): self.count += 1

EVAL_PRINT_EVERY = 50  # print every 50 function calls

def make_objective(counter: EvalCounter):
    t0 = time.perf_counter()
    def obj(x):
        counter.inc()
        t1, t2, t3, s1, s2, s3, s4 = x

        # soft penalties to keep reversal times within suggested windows
        pen = 0.0
        for ti, (tmin, tmax) in zip([t1, t2, t3], [(T1_MIN, T1_MAX), (T2_MIN, T2_MAX), (T3_MIN, T3_MAX)]):
            if ti < tmin: pen += (tmin - ti)**2
            if ti > tmax: pen += (ti - tmax)**2

        # angles soft penalty (hard limits enforced later in command clamp)
        for s in [s1, s2, s3, s4]:
            if s < SIGMA_MIN_DEG: pen += (SIGMA_MIN_DEG - s)**2
            if s > SIGMA_MAX_DEG: pen += (s - SIGMA_MAX_DEG)**2

        # enforce ordering softly
        if not (t1 <= t2 <= t3):
            ts = np.array([t1, t2, t3]); pen += 10.0*np.sum((ts - np.sort(ts))**2)

        # simulate with limiter + deploy rule
        final_alt = simulate_profile(x)
        J = -final_alt + 1e-6*pen
        # >>> quick progress print <<<
        if counter.count % EVAL_PRINT_EVERY == 0:
            elapsed = time.perf_counter() - t0
            t1, t2, t3, s1, s2, s3, s4 = x
            print(f"[OBJ] eval={counter.count:5d}  J={J: .3f}  "
                  f"t=[{t1:6.1f},{t2:6.1f},{t3:6.1f}]  "
                  f"s=[{s1:6.1f},{s2:6.1f},{s3:6.1f},{s4:6.1f}]  "
                  f"elapsed={elapsed:6.1f}s")
        return J
    return obj

def hooke_jeeves(objective, x0, step0=50.0, step_min=1e-2, alpha=2.0, gamma=0.5, max_evals=5000):
    x = x0.copy()
    n = len(x)
    step = np.ones(n)*step0
    fx = objective(x); evals = 1
    while np.max(step) > step_min and evals < max_evals:
        improved = False
        x_new = x.copy(); f_best = fx
        for i in range(n):
            for d in [step[i], -step[i]]:
                cand = x_new.copy(); cand[i] += d
                f = objective(cand); evals += 1
                if f < f_best:
                    x_new, f_best, improved = cand, f, True
        if improved:
            print(f"[HJ] improve  f={f_best: .3f}  step_max={np.max(step):.2f}")
            d = x_new - x
            x_pat = x_new + alpha*d
            f_pat = objective(x_pat); evals += 1
            if f_pat < f_best:
                x, fx = x_pat, f_pat
            else:
                x, fx = x_new, f_best
        else:
            step *= gamma
            print(f"[HJ] shrink   step_max={np.max(step):.2f}  f={fx: .3f}")

    return x, fx, evals

# ============================================================
# REPORTING & PLOTS
# ============================================================
def print_params(x):
    t1, t2, t3, s1, s2, s3, s4 = x
    t1, t2, t3 = build_times_sorted(t1, t2, t3)
    print(f"  t1,t2,t3  = {t1:7.2f}, {t2:7.2f}, {t3:7.2f}  [s]")
    print(f"  sigmas    = {s1:7.2f}, {s2:7.2f}, {s3:7.2f}, {s4:7.2f}  [deg]")

def _collect_hist(x):
    final_alt, hist, deployed, t_dep = simulate_profile(x, return_history=True)
    import numpy as np
    out = {
        "t": np.array(hist["t"]),
        "h": np.array(hist["h"])/1e3,  # km
        "V": np.array(hist["V"]),
        "q": np.array(hist["q"]),
        "mach": np.array(hist["mach"]),
        "gam": np.rad2deg(np.array(hist["gam"])),
        "psi": np.rad2deg(np.array(hist["psi"])),
        "sig_cmd": np.rad2deg(np.array(hist["sigma_cmd"])),
        "sig": np.rad2deg(np.array(hist["sigma"])),
        "sigdot": np.rad2deg(np.array(hist["sigma_dot"])),
        "sigddot": np.rad2deg(np.array(hist["sigma_ddot"])),
        "final_alt": final_alt/1e3,  # km
        "t_dep": t_dep,
    }
    t1, t2, t3, *_ = x
    out["t1"], out["t2"], out["t3"] = build_times_sorted(t1, t2, t3)
    return out

def plot_states(x, title_prefix="Profile"):
    d = _collect_hist(x)
    import matplotlib.pyplot as plt

    def vline(ax, ti, lab):
        if ti is None: return
        ax.axvline(ti, ls="--", alpha=0.5)
        ytop = ax.get_ylim()[1]
        ax.text(ti, ytop*0.92, lab, rotation=90, va='top', ha='left', alpha=0.7, fontsize=8)

    def dep_marker(ax):
        if d["t_dep"] is not None:
            ax.axvline(d["t_dep"], ls=":", alpha=0.9)
            ytop = ax.get_ylim()[1]
            ax.text(d["t_dep"], ytop*0.85, "DEPLOY", rotation=90, va='top', ha='left', fontweight='bold', fontsize=8)

    fig = plt.figure(figsize=(12, 14))
    fig.suptitle(f"{title_prefix} — States (final h = {d['final_alt']:.1f} km)", y=0.98, fontsize=14)
    gs = fig.add_gridspec(6, 1, hspace=0.35)
    axes = [fig.add_subplot(gs[i, 0]) for i in range(6)]

    ax = axes[0]
    ax.plot(d["t"], d["h"]); ax.set_xlabel("t [s]"); ax.set_ylabel("Altitude [km]")
    vline(ax,d["t1"],"t1"); vline(ax,d["t2"],"t2"); vline(ax,d["t3"],"t3"); dep_marker(ax); ax.grid(True)

    ax = axes[1]
    ax.plot(d["t"], d["V"]); ax.set_xlabel("t [s]"); ax.set_ylabel("Velocity [m/s]")
    vline(ax,d["t1"],"t1"); vline(ax,d["t2"],"t2"); vline(ax,d["t3"],"t3"); dep_marker(ax); ax.grid(True)

    ax = axes[2]
    ax.plot(d["t"], d["q"]); ax.set_xlabel("t [s]"); ax.set_ylabel("q [Pa]")
    ax.fill_between(d["t"], 300.0, 800.0, alpha=0.1, step="pre", label="deploy q-band")
    vline(ax,d["t1"],"t1"); vline(ax,d["t2"],"t2"); vline(ax,d["t3"],"t3"); dep_marker(ax); ax.grid(True)

    ax = axes[3]
    ax.plot(d["t"], d["mach"]); ax.set_xlabel("t [s]"); ax.set_ylabel("Mach [-]")
    ax.fill_between(d["t"], 1.4, 2.2, alpha=0.1, step="pre", label="deploy M-band")
    vline(ax,d["t1"],"t1"); vline(ax,d["t2"],"t2"); vline(ax,d["t3"],"t3"); dep_marker(ax); ax.grid(True)

    ax = axes[4]
    ax.plot(d["t"], d["gam"]); ax.set_xlabel("t [s]"); ax.set_ylabel("γ [deg]")
    vline(ax,d["t1"],"t1"); vline(ax,d["t2"],"t2"); vline(ax,d["t3"],"t3"); dep_marker(ax); ax.grid(True)

    ax = axes[5]
    ax.plot(d["t"], d["psi"]); ax.set_xlabel("t [s]"); ax.set_ylabel("ψ [deg]")
    vline(ax,d["t1"],"t1"); vline(ax,d["t2"],"t2"); vline(ax,d["t3"],"t3"); dep_marker(ax); ax.grid(True)

    plt.show()

def plot_sigma(x, title_prefix="Profile"):
    d = _collect_hist(x)

    fig, ax = plt.subplots(figsize=(12, 5))
    fig.suptitle(f"{title_prefix} — Bank & Rates", y=0.98, fontsize=14)

    ax.plot(d["t"], d["sig_cmd"], lw=1.2, label="σ_cmd [deg]")
    ax.plot(d["t"], d["sig"], lw=1.8, label="σ (applied) [deg]")
    ax.plot(d["t"], d["sigdot"], lw=1.0, label="σ̇ [deg/s]")
    ax.plot(d["t"], d["sigddot"], lw=1.0, label="σ̈ [deg/s²]")

    # limit lines
    ax.axhline(+SIGMA_DOT_MAX_DEG_S, ls="--", alpha=0.4)
    ax.axhline(-SIGMA_DOT_MAX_DEG_S, ls="--", alpha=0.4)
    ax.axhline(+SIGMA_DDOT_MAX_DEG_S2, ls="--", alpha=0.4)
    ax.axhline(-SIGMA_DDOT_MAX_DEG_S2, ls="--", alpha=0.4)

    ax.set_xlabel("t [s]")
    ax.set_ylabel("deg / deg·s⁻¹ / deg·s⁻²")
    ax.grid(True)
    ax.legend(ncol=4, fontsize=9, loc="upper right")

    # markers
    for ti, lab in [(d["t1"],"t1"), (d["t2"],"t2"), (d["t3"],"t3")]:
        if ti is not None:
            ax.axvline(ti, ls="--", alpha=0.5)
            ytop = ax.get_ylim()[1]
            ax.text(ti, ytop*0.92, lab, rotation=90, va='top', ha='left', alpha=0.7, fontsize=8)

    if d["t_dep"] is not None:
        ax.axvline(d["t_dep"], ls=":", alpha=0.9)
        ytop = ax.get_ylim()[1]
        ax.text(d["t_dep"], ytop*0.85, "DEPLOY", rotation=90, va='top', ha='left', fontweight='bold', fontsize=8)

    plt.show()


# ============================================================
# RUN OPTIMIZERS & COMPARE
# ============================================================
def run_comparison():
    x_init = x0.copy()

    # Nelder–Mead
    nm_counter = EvalCounter()
    nm_obj = make_objective(nm_counter)
    t0 = time.perf_counter()
    res_nm = minimize(nm_obj, x_init, method="Nelder-Mead",
                      options=dict(maxiter=2000, xatol=1e-3, fatol=1e-3, disp=False))
    nm_time = time.perf_counter() - t0
    nm_x, nm_J, nm_evals = res_nm.x, res_nm.fun, nm_counter.count

    # Hooke–Jeeves
    hj_counter = EvalCounter()
    hj_obj = make_objective(hj_counter)
    t0 = time.perf_counter()
    hj_x, hj_J, hj_evals = hooke_jeeves(hj_obj, x_init, step0=80.0, step_min=0.5, alpha=1.8, gamma=0.5, max_evals=8000)
    hj_time = time.perf_counter() - t0

    if hj_J < nm_J:
        best_name, best_x, best_J = "Hooke–Jeeves", hj_x, hj_J
    else:
        best_name, best_x, best_J = "Nelder–Mead", nm_x, nm_J

    print("\n=== Optimization comparison (J = - final altitude at deploy/impact) ===")
    print(f"Nelder–Mead:  J = {nm_J: .3f},   evals = {nm_evals},   time = {nm_time: .3f}s")
    print(f"Hooke–Jeeves: J = {hj_J: .3f},   evals = {hj_evals},   time = {hj_time: .3f}s")
    print(f"\nBest: {best_name}")
    print_params(best_x)

    return {"nm": dict(x=nm_x, J=nm_J, evals=nm_evals, time=nm_time, success=res_nm.success),
            "hj": dict(x=hj_x, J=hj_J, evals=hj_evals, time=hj_time),
            "best": dict(name=best_name, x=best_x, J=best_J)}

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    results = run_comparison()
    best_name = results["best"]["name"]
    best_x    = results["best"]["x"]
    print("\n=== Best parameters ===")
    print_params(best_x)

    # Nelder–Mead
    plot_states(results["nm"]["x"], title_prefix="Nelder–Mead")
    plot_sigma(results["nm"]["x"],  title_prefix="Nelder–Mead")

    # Hooke–Jeeves
    plot_states(results["hj"]["x"], title_prefix="Hooke–Jeeves")
    plot_sigma(results["hj"]["x"],  title_prefix="Hooke–Jeeves")

