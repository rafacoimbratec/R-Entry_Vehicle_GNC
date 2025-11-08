"""
3DOF Reference Trajectory Generation - Design Map Sweep
Author: You
Description:
  - Planar 3DOF point-mass reentry on Mars
  - Vertical L/D bank schedule (no reversals)
  - Exponential atmosphere + Sutton–Graves heating (for monitoring)
  - Deploy rule: first time V <= 450 m/s OR h <= 6000 m
  - Design-map sweep over EFPA and LDv schedule endpoints with Table-2 style filters
Outputs:
  - design_map_results.csv (all candidates and PASS flag)
  - Prints the selected reference (highest deploy altitude among PASS)
"""

from dataclasses import dataclass
import numpy as np
import pandas as pd
import math
from spherical_metrics import spherical_metrics
import matplotlib.pyplot as plt

# -----------------------
# Planet/vehicle/IC/target/constraints
# -----------------------
@dataclass
class MarsConstants:
    radius: float = 3396.2e3        # [m]
    mu: float = 4.282837e13         # [m^3/s^2]
    omega: float = 7.088e-5         # [rad/s]
    # Simple exponential atmosphere
    H_s: float = 11100.0            # [m]
    rho0: float = 0.020             # [kg/m^3]
    # Thermodynamics (for Mach estimate)
    specific_heat_ratio: float = 1.29
    R_star: float = 8314.32         # [J/(kmol*K)]
    M_CO2: float = 44.01            # [kg/kmol]

    @property
    def R_s(self) -> float:
        return self.R_star / self.M_CO2

@dataclass
class VehicleProperties:
    L_over_D: float = 0.24          # nominal L/D
    beta: float = 135.0             # [kg/m^2] ballistic coefficient
    # Heating correlation (monitoring only)
    N: float = 0.5
    M: float = 3
    k_heat_flux: float = 5.3697e-5  # SI -> W/m^2 with rho^N * V^M

@dataclass
class InitialConditions:
    h: float = 120e3
    theta: float = -176.40167 * np.pi/180  # Initial longitude [rad]
    phi: float = -21.3 * np.pi/180  # Initial latitude [rad]
    V: float = 4700
    gamma: float = -15.0 * np.pi/180
    psi: float = -2.8758 * np.pi/180  # Initial heading angle [rad]

@dataclass
class TargetConditions:
    theta: float = -175.8 * np.pi/180
    phi: float = 0.276 * np.pi/180
    V: float = 450.0
    h: float = 2480.0
    Rgo_target: float = 5e3

@dataclass
class PathConstraints:
    A_max: float = 5.0          # [g] converted inside if needed
    q_max: float = 13e3         # [Pa]
    Qdot_max: float = 500e3     # [W/m^2]

# -----------------------
# Atmosphere + state type
# -----------------------
class MarsAtmosphere:
    def __init__(self, mars: MarsConstants):
        self.mars = mars

    def density(self, h: float) -> float:
        return self.mars.rho0 * np.exp(-max(h, 0.0) / self.mars.H_s)

    def temperature(self, h: float) -> float:
        # Simple polynomial fit (coarse), K
        h_km = max(h, 0.0) / 1e3
        return 1.4e-13 * h_km**3 - 8.85e-9 * h_km**2 - 1.245e-3 * h_km + 205.36

class State:
    """[r, theta, phi, V, gamma, psi]"""
    def __init__(self, r, theta, phi, V, gamma, psi):
        self.r, self.theta, self.phi = r, theta, phi
        self.V, self.gamma, self.psi = V, gamma, psi

    @property
    def h(self):  # altitude
        return self.r - 3396.2e3

    def to_array(self):
        return np.array([self.r, self.theta, self.phi, self.V, self.gamma, self.psi])

    @classmethod
    def from_array(cls, y):
        return cls(y[0], y[1], y[2], y[3], y[4], y[5])

# -----------------------
# 3DOF dynamics (planar)
# -----------------------
class ReentryDynamics:
    def __init__(self, mars: MarsConstants, veh: VehicleProperties, atm: MarsAtmosphere):
        self.mars, self.veh, self.atm = mars, veh, atm

    def equations_of_motion(self, state: State, sigma: float) -> np.ndarray:
        r, theta, phi, V, gamma, psi = state.to_array()
        h = r - self.mars.radius
        rho = self.atm.density(h)
        g = self.mars.mu / r**2
        q = 0.5 * rho * V**2

        # Drag & lift accelerations (D = q/beta; L = (L/D)*D)
        D = q / self.veh.beta
        L = self.veh.L_over_D * D

        Vh = V * np.cos(gamma)

        r_dot     = V * np.sin(gamma)
        theta_dot = (Vh * np.sin(psi)) / (r * np.cos(phi))
        phi_dot   = (Vh * np.cos(psi)) / r
        V_dot     = -D - g * np.sin(gamma) + self.mars.omega**2 * r * np.cos(phi) * \
                    (np.sin(gamma) * np.cos(phi) - np.cos(gamma) * np.sin(phi) * np.cos(psi))
        gamma_dot = (L * np.cos(sigma) / max(V,1e-6)) + (V / r - g / max(V,1e-6)) * np.cos(gamma) + \
                    2 * self.mars.omega * np.cos(phi) * np.sin(psi) + \
                    (self.mars.omega**2 * r / max(V,1e-6)) * np.cos(phi) * \
                    (np.cos(gamma) * np.cos(phi) + np.sin(gamma) * np.sin(phi) * np.cos(psi))
        psi_dot   = (L * np.sin(sigma)) / (max(V,1e-6) * max(np.cos(gamma),1e-6)) + \
                    (Vh / r) * np.sin(psi) * np.tan(phi) - \
                    2 * self.mars.omega * (np.tan(gamma) * np.cos(phi) * np.cos(psi) - np.sin(phi)) + \
                    (self.mars.omega**2 * r / (max(V,1e-6) * max(np.cos(gamma),1e-6))) * np.sin(phi) * np.cos(phi) * np.sin(psi)

        return np.array([r_dot, theta_dot, phi_dot, V_dot, gamma_dot, psi_dot])

    def rk4_step(self, state: State, sigma: float, dt: float) -> State:
        y = state.to_array()
        k1 = self.equations_of_motion(state, sigma)
        k2 = self.equations_of_motion(State.from_array(y + 0.5*dt*k1), sigma)
        k3 = self.equations_of_motion(State.from_array(y + 0.5*dt*k2), sigma)
        k4 = self.equations_of_motion(State.from_array(y + dt*k3),  sigma)
        y_next = y + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        return State.from_array(y_next)

    def aero_monitor(self, state: State):
        h = state.h
        rho = self.atm.density(h)
        q   = 0.5 * rho * state.V**2
        D   = q / self.veh.beta
        L   = self.veh.L_over_D * D
        g0  = 9.80665
        A_g = np.sqrt(D**2 + L**2) / g0
        Qdot = self.veh.k_heat_flux * (rho**self.veh.N) * (state.V**self.veh.M)
        return {"rho": rho, "q": q, "D": D, "L": L, "A_g": A_g, "Qdot": Qdot}

# -----------------------
# Reentry simulation loop
# -----------------------
class ReentrySimulation:
    def __init__(self, mars, veh, ic, target, constraints):
        self.mars, self.veh, self.ic, self.target, self.constraints = mars, veh, ic, target, constraints
        self.atm = MarsAtmosphere(mars)
        self.dyn = ReentryDynamics(mars, veh, self.atm)

    def run(self, start_time=0.0, stop_time=7000.0, time_step=0.1, guidance=None, bank_angle=0.0, guidance_dt=0.5):
        r0 = self.ic.h + self.mars.radius
        state = State(r0, self.ic.theta, self.ic.phi, self.ic.V, self.ic.gamma, self.ic.psi)

        t = np.arange(start_time, stop_time, time_step)
        n = len(t)
        res = {k: np.zeros(n) for k in ["t","r","h","theta","phi","V","gamma","psi","sigma","q","A","Qdot","rho"]}
        res.update({"Mach": np.zeros(n)})

        # reference for downrange/crossrange calculations (ENU at initial location)
        phi0 = self.ic.phi
        theta0 = self.ic.theta
        psi0 = self.ic.psi

        next_guid = start_time
        last_sigma = bank_angle if guidance is None else 0.0

        for i, ti in enumerate(t):
            res["t"][i] = ti

            if guidance is not None and ti + 1e-9 >= next_guid:
                last_sigma = guidance.update(ti, state, self.target, self.mars.radius,
                                             atmosphere=self.atm, constraints=self.constraints)
                next_guid += guidance_dt
            sigma = last_sigma

            # store
            res["r"][i], res["theta"][i], res["phi"][i] = state.r, state.theta, state.phi
            res["V"][i], res["gamma"][i], res["psi"][i] = state.V, state.gamma, state.psi
            res["sigma"][i] = sigma
            res["h"][i] = state.h

            am = self.dyn.aero_monitor(state)
            res["rho"][i], res["q"][i], res["A"][i], res["Qdot"][i] = am["rho"], am["q"], am["A_g"], am["Qdot"]


            # crude Mach estimate
            a = math.sqrt(self.mars.specific_heat_ratio * self.mars.R_s * self.atm.temperature(state.h))
            mach_now = state.V / max(a, 1e-6)
            res["Mach"][i] = mach_now

            # deploy/stop condition requested: (Mach in [1.4,2.2] AND q in [300,800]) OR h <= 0
            q_now = res["q"][i]
            mach_trigger = (1.4 <= mach_now <= 2.2)
            q_trigger = (300.0 <= q_now <= 800.0)
            if (mach_trigger and q_trigger) or (state.h <= 0.0):
                # compute final spherical metrics once (no per-step approximation)
                lat0 = self.ic.phi
                lon0 = self.ic.theta
                lat_final = state.phi
                lon_final = state.theta
                lat_t = self.target.phi
                lon_t = self.target.theta
                try:
                    _, _, _, RC, RD, _, _ = spherical_metrics(lat0, lon0, lat_final, lon_final, lat_final, lon_final, self.mars.radius)
                    final_cross = RC
                    final_down = RD
                except Exception:
                    final_cross = float('nan')
                    final_down = float('nan')

                # store final scalar metrics
                res["final_downrange_m"] = final_down
                res["final_crossrange_m"] = final_cross

                for k in list(res.keys()):
                    # truncate only array entries
                    if isinstance(res[k], np.ndarray):
                        res[k] = res[k][:i+1]
                break

            # integrate
            state = self.dyn.rk4_step(state, sigma, time_step)

        return res

# -----------------------
# Bank-schedule guidance (vertical L/D vs speed)
# -----------------------
class BankScheduleGuidance:
    def __init__(self, L_over_D_nom, V_hi, V_lo, LDv_hi, LDv_lo):
        self.LDnom = float(L_over_D_nom)
        self.V_hi, self.V_lo = float(V_hi), float(V_lo)
        self.LDv_hi, self.LDv_lo = float(LDv_hi), float(LDv_lo)

    def LDv(self, V):
        if V >= self.V_hi: return self.LDv_hi
        if V <= self.V_lo: return self.LDv_lo
        a = (V - self.V_lo) / (self.V_hi - self.V_lo)
        return self.LDv_lo + a*(self.LDv_hi - self.LDv_lo)

    def update(self, t, state, target, Rplanet, atmosphere=None, constraints=None):
        LDv_now = self.LDv(state.V)
        if self.LDnom <= 1e-9:  # no lift
            return np.deg2rad(90.0)
        c = np.clip(LDv_now / self.LDnom, -1.0, 1.0)
        #print(f"t={t:.1f} s, V={state.V:.1f} m/s, LDv={LDv_now:.3f}, bank angle σ={math.degrees(math.acos(c)):.1f} deg")
        return math.acos(c)  # [rad]

# (Removed legacy design-map and candidate plotting helpers to keep this
# module focused on the EFPA/LDv constraint sweep and plotting utilities.)

# -----------------------
# Run sweep
# -----------------------
def _expand_range_or_list(x):
    """If x is a 3-tuple (start, stop, step) return np.arange, otherwise
    convert to list and return as-is."""
    if x is None:
        return []
    if isinstance(x, (list, tuple)) and len(x) == 3 and all(isinstance(v, (int, float)) for v in x):
        return list(np.arange(x[0], x[1] + 1e-9, x[2]))
    # otherwise assume iterable of values
    return list(x)


def run_constraint_sweep(sim: ReentrySimulation,
                         efpa_grid=(-16.0, -15.0, 0.2),
                         early_bank_deg=(45, 115, 10),
                         early_V=(4500, 6000, 500),
                         late_bank_deg=(40, 60, 5),
                         late_V=(2000, 3500, 250),
                         time_step=0.1, guidance_dt=0.5,
                         out_csv="efpa_bank_constraint_sweep.csv"):
    """Simplified sweep: for each EFPA and LDv pair, simulate using the existing
    BankScheduleGuidance and report whether the resulting bank(t) trajectory
    respects q, A_g and Qdot constraints in sim.constraints.

    Keeps the original bank-angle method unchanged and eliminates unrelated
    design-map filtering/plotting for a focused test.
    """
    efpas = np.arange(efpa_grid[0], efpa_grid[1] + 1e-9, efpa_grid[2])
    # interpret bank-angle ranges (deg) and velocity ranges
    early_bank_list = _expand_range_or_list(early_bank_deg)
    late_bank_list = _expand_range_or_list(late_bank_deg)
    early_V_list = _expand_range_or_list(early_V)
    late_V_list = _expand_range_or_list(late_V)

    # convert bank angles (deg) to LDv values using LD_nom * cos(bank_rad)
    LD_nom = float(sim.veh.L_over_D)
    LDv_hi_vals = [LD_nom * math.cos(math.radians(b)) for b in early_bank_list]
    LDv_lo_vals = [LD_nom * math.cos(math.radians(b)) for b in late_bank_list]

    rows = []
    runs = []
    for efpa_deg in efpas:
        for LDv_hi in LDv_hi_vals:
            for V_hi in early_V_list:
                for LDv_lo in LDv_lo_vals:
                    for V_lo in late_V_list:
                        # build IC with requested EFPA
                        ic_mod = InitialConditions(h=sim.ic.h, theta=sim.ic.theta, phi=sim.ic.phi,
                                                   V=sim.ic.V, gamma=np.deg2rad(efpa_deg), psi=sim.ic.psi)
                        sim_local = ReentrySimulation(sim.mars, sim.veh, ic_mod, sim.target, sim.constraints)
                        g = BankScheduleGuidance(sim.veh.L_over_D, V_hi, V_lo, LDv_hi, LDv_lo)

                        res = sim_local.run(time_step=time_step, guidance=g, guidance_dt=guidance_dt)
                        if len(res["t"]) == 0:
                            # no integration, treat as fail
                            max_q = float('nan'); max_A = float('nan'); max_Qdot = float('nan')
                            pass_q = pass_A = pass_Qdot = False
                        else:
                            max_q = float(np.max(res["q"]))
                            max_A = float(np.max(res["A"]))
                            max_Qdot = float(np.max(res["Qdot"]))
                            pass_q = (max_q <= sim.constraints.q_max)
                            pass_A = (max_A <= sim.constraints.A_max)
                            pass_Qdot = (max_Qdot <= sim.constraints.Qdot_max)

                        # compute a normalized constraint score (lower is better)
                        try:
                            score = (max_q / sim.constraints.q_max) + (max_A / sim.constraints.A_max) + (max_Qdot / sim.constraints.Qdot_max)
                        except Exception:
                            score = float('nan')

                        rows.append({
                            "efpa_deg": efpa_deg,
                            "LDv_hi": LDv_hi,
                            "LDv_lo": LDv_lo,
                            "V_hi": V_hi,
                            "V_lo": V_lo,
                            "max_q_Pa": max_q,
                            "max_A_g": max_A,
                            "max_Qdot_Wm2": max_Qdot,
                            "score": score,
                            "PASS_q": bool(pass_q),
                            "PASS_A_g": bool(pass_A),
                            "PASS_Qdot": bool(pass_Qdot),
                            "PASS_all": bool(pass_q and pass_A and pass_Qdot)
                        })

                        runs.append({
                            "efpa_deg": efpa_deg,
                            "LDv_hi": LDv_hi,
                            "LDv_lo": LDv_lo,
                            "V_hi": V_hi,
                            "V_lo": V_lo,
                            "res": res,
                            "PASS_all": bool(pass_q and pass_A and pass_Qdot),
                            "score": score
                        })
                        #print(f"Run EFPA={efpa_deg}°, LDv_hi={LDv_hi}, LDv_lo={LDv_lo} -> ")

    df = pd.DataFrame(rows)
    n_pass = int(df["PASS_all"].sum())

    # If score is available, save only the best (lowest) scoring run to CSV
    if "score" in df.columns and not df["score"].isnull().all():
        df_valid = df.dropna(subset=["score"])
        if not df_valid.empty:
            best_idx = df_valid["score"].idxmin()
            best_row = df_valid.loc[[best_idx]]
            best_row.to_csv(out_csv, index=False)
            print(f"Constraint sweep complete: {len(df)} runs, {n_pass} passed all constraints. Saved best run to: {out_csv}")
            return df, runs
    # fallback: save entire dataframe if no score column
    df.to_csv(out_csv, index=False)
    print(f"Constraint sweep complete: {len(df)} runs, {n_pass} passed all constraints. Saved: {out_csv}")
    return df, runs


def run_efpa_family(sim: ReentrySimulation, efpa_grid, V_hi, V_lo, LDv_hi, LDv_lo, time_step=0.1, guidance_dt=0.5):
    """Run all EFPA values while keeping bank schedule fixed (V_hi/V_lo and LDv_hi/LDv_lo).
    Returns list of run dicts similar to sweep runs (with res and PASS_all).
    """
    efpas = np.arange(efpa_grid[0], efpa_grid[1] + 1e-9, efpa_grid[2])
    runs = []
    for efpa_deg in efpas:
        ic_mod = InitialConditions(h=sim.ic.h, theta=sim.ic.theta, phi=sim.ic.phi,
                                   V=sim.ic.V, gamma=np.deg2rad(efpa_deg), psi=sim.ic.psi)
        sim_local = ReentrySimulation(sim.mars, sim.veh, ic_mod, sim.target, sim.constraints)
        g = BankScheduleGuidance(sim.veh.L_over_D, V_hi, V_lo, LDv_hi, LDv_lo)
        res = sim_local.run(time_step=time_step, guidance=g, guidance_dt=guidance_dt)
        if len(res["t"]) == 0:
            max_q = float('nan'); max_A = float('nan'); max_Qdot = float('nan')
            pass_q = pass_A = pass_Qdot = False
        else:
            max_q = float(np.max(res["q"]))
            max_A = float(np.max(res["A"]))
            max_Qdot = float(np.max(res["Qdot"]))
            pass_q = (max_q <= sim.constraints.q_max)
            pass_A = (max_A <= sim.constraints.A_max)
            pass_Qdot = (max_Qdot <= sim.constraints.Qdot_max)
        try:
            score = (max_q / sim.constraints.q_max) + (max_A / sim.constraints.A_max) + (max_Qdot / sim.constraints.Qdot_max)
        except Exception:
            score = float('nan')
        runs.append({
            "efpa_deg": efpa_deg,
            "LDv_hi": LDv_hi,
            "LDv_lo": LDv_lo,
            "V_hi": V_hi,
            "V_lo": V_lo,
            "res": res,
            "max_q_Pa": max_q,
            "max_A_g": max_A,
            "max_Qdot_Wm2": max_Qdot,
            "PASS_all": bool(pass_q and pass_A and pass_Qdot),
            "score": score
        })
    return runs


def run_bank_schedule_reachable_set(sim: ReentrySimulation,
                                    efpa_deg=-12.0,
                                    early_bank_deg=(45, 115, 10),
                                    early_V=(4500, 6000, 500),
                                    late_bank_deg=(40, 60, 5),
                                    late_V=(2000, 3500, 250),
                                    time_step=0.1, guidance_dt=0.5,
                                    out_csv="reachable_set.csv"):
    """Fix EFPA to efpa_deg and sweep bank-schedule endpoints to produce a reachable
    set (downrange, crossrange) and final altitude for each trajectory. The score
    is computed as the sum of normalized peak constraint costs (q/q_max + A/A_max + Qdot/Qdot_max).
    """
    # single efpa
    efpa = float(efpa_deg)
    # interpret bank-angle ranges
    early_bank_list = _expand_range_or_list(early_bank_deg)
    late_bank_list = _expand_range_or_list(late_bank_deg)
    early_V_list = _expand_range_or_list(early_V)
    late_V_list = _expand_range_or_list(late_V)

    LD_nom = float(sim.veh.L_over_D)
    LDv_hi_vals = [LD_nom * math.cos(math.radians(b)) for b in early_bank_list]
    LDv_lo_vals = [LD_nom * math.cos(math.radians(b)) for b in late_bank_list]

    rows = []
    runs = []
    for LDv_hi in LDv_hi_vals:
        for V_hi in early_V_list:
            for LDv_lo in LDv_lo_vals:
                for V_lo in late_V_list:
                    ic_mod = InitialConditions(h=sim.ic.h, theta=sim.ic.theta, phi=sim.ic.phi,
                                               V=sim.ic.V, gamma=np.deg2rad(efpa), psi=sim.ic.psi)
                    sim_local = ReentrySimulation(sim.mars, sim.veh, ic_mod, sim.target, sim.constraints)
                    g = BankScheduleGuidance(sim.veh.L_over_D, V_hi, V_lo, LDv_hi, LDv_lo)

                    res = sim_local.run(time_step=time_step, guidance=g, guidance_dt=guidance_dt)
                    if len(res.get("t", [])) == 0:
                        max_q = float('nan'); max_A = float('nan'); max_Qdot = float('nan')
                        pass_q = pass_A = pass_Qdot = False
                        final_down = float('nan'); final_cross = float('nan'); final_h = float('nan')
                    else:
                        max_q = float(np.max(res["q"]))
                        max_A = float(np.max(res["A"]))
                        max_Qdot = float(np.max(res["Qdot"]))
                        pass_q = (max_q <= sim.constraints.q_max)
                        pass_A = (max_A <= sim.constraints.A_max)
                        pass_Qdot = (max_Qdot <= sim.constraints.Qdot_max)
                        # compute final downrange/crossrange using spherical metrics (single call)
                        try:
                            lat0 = sim_local.ic.phi
                            lon0 = sim_local.ic.theta
                            lat_final = float(res["phi"][-1])
                            lon_final = float(res["theta"][-1])
                            lat_t = sim_local.target.phi
                            lon_t = sim_local.target.theta
                            _, _, _, RC, RD, _, _ = spherical_metrics(lat0, lon0, lat_final, lon_final, lat_t, lon_t, sim_local.mars.radius)
                            final_cross = float(RC)
                            final_down = float(RD)
                        except Exception:
                            final_cross = float('nan')
                            final_down = float('nan')
                        final_h = float(res["h"][-1])

                    # score: sum of normalized constraint peaks (lower is better)
                    try:
                        score = (max_q / sim.constraints.q_max) + (max_A / sim.constraints.A_max) + (max_Qdot / sim.constraints.Qdot_max)
                    except Exception:
                        score = float('nan')

                    rows.append({
                        "efpa_deg": efpa,
                        "LDv_hi": LDv_hi,
                        "LDv_lo": LDv_lo,
                        "V_hi": V_hi,
                        "V_lo": V_lo,
                        "max_q_Pa": max_q,
                        "max_A_g": max_A,
                        "max_Qdot_Wm2": max_Qdot,
                        "score": score,
                        "PASS_all": bool(pass_q and pass_A and pass_Qdot),
                        "final_down_m": final_down,
                        "final_cross_m": final_cross,
                        "final_h_m": final_h
                    })

                    runs.append({
                        "efpa_deg": efpa,
                        "LDv_hi": LDv_hi,
                        "LDv_lo": LDv_lo,
                        "V_hi": V_hi,
                        "V_lo": V_lo,
                        "res": res,
                        "PASS_all": bool(pass_q and pass_A and pass_Qdot),
                        "score": score,
                        "final_down_m": final_down,
                        "final_cross_m": final_cross,
                        "final_h_m": final_h
                    })

    df = pd.DataFrame(rows)
    df.to_csv(out_csv, index=False)
    print(f"Reachable-set sweep complete: {len(df)} runs. Saved: {out_csv}")
    return df, runs


def plot_reachable_set(df, title=None, figsize=(8,6), out_png="reachable_set.png"):
    """Scatter plot CrossRange vs Downrange colored by final altitude.
    Expects columns final_down_m, final_cross_m, final_h_m in dataframe.
    """
    df_valid = df.dropna(subset=["final_down_m", "final_cross_m", "final_h_m"]).copy()
    if df_valid.empty:
        print("No valid runs to plot.")
        return None
    x_km = df_valid["final_down_m"].values / 1000.0
    y_km = df_valid["final_cross_m"].values / 1000.0
    z_km = df_valid["final_h_m"].values / 1000.0

    fig, ax = plt.subplots(figsize=figsize)
    sc = ax.scatter(x_km, y_km, c=z_km, cmap='viridis', s=40, edgecolors='k')
    cb = fig.colorbar(sc, ax=ax)
    cb.set_label('Final altitude [km]')
    ax.set_xlabel('Downrange [km]')
    ax.set_ylabel('Crossrange [km]')
    if title:
        ax.set_title(title)
    ax.grid(True)
    fig.savefig(out_png, dpi=200)
    print(f"Saved reachable set figure: {out_png}")
    plt.show()
    return fig, ax


def plot_sweep_states_and_constraints(runs, sim=None, figsize=(12,10),
                                      states_file="efpa_states.png",
                                      constraints_file="efpa_constraints.png",
                                      title: str = None):
    """Create two figures: stacked state traces and path-constraint traces.

    - States figure: altitude, speed, flight-path angle, bank angle
    - Constraints figure: q, Qdot, A (with markers at maxima)

    Each run is colored by PASS_all (blue=pass, gray=fail) and labeled by EFPA/LDv.
    """
    if len(runs) == 0:
        print("No runs to plot.")
        return None

    # --- States figure ---
    state_vars = [
        ("h", "Altitude [km]", lambda r: np.array(r)/1000.0),
        ("V", "Speed [m/s]", lambda r: np.array(r)),
        ("gamma", "FPA γ [deg]", lambda r: np.degrees(np.array(r))),
        ("sigma", "Bank angle σ [deg]", lambda r: np.degrees(np.array(r)))
    ]

    fig1, axes1 = plt.subplots(len(state_vars), 1, figsize=(figsize[0], figsize[1]*0.6), sharex=True)
    cmap = plt.get_cmap('tab10')
    for i, run in enumerate(runs):
        res = run.get("res")
        if res is None:
            continue
        t = res["t"]
        base_color = cmap(i % cmap.N)
        passed = bool(run.get('PASS_all', False))
        # passed runs colored, failed runs desaturated via alpha
        color = base_color if passed else (*base_color[:3], 0.45)
        lw = 1.8 if passed else 1.0
        label = f"EFPA={run['efpa_deg']}°, LDv_hi={run['LDv_hi']}, LDv_lo={run['LDv_lo']}"
        for ax, (var, ylabel, transform) in zip(axes1, state_vars):
            if var not in res:
                continue
            y = transform(res[var])
            ax.plot(t, y, label=label, color=color, linewidth=lw)
            ax.set_ylabel(ylabel)
            ax.grid(True)

    axes1[-1].set_xlabel("Time [s]")
    if title:
        fig1.suptitle(title)
    # do not show legend (too many traces); reserve space for title
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig1.savefig(states_file, dpi=150)
    print(f"Saved states figure: {states_file}")

    # --- Constraints figure ---
    cons_vars = [
        ("q", "Dynamic pressure q [Pa]", lambda r: np.array(r)),
        ("Qdot", "Stagnation heat rate [W/m^2]", lambda r: np.array(r)),
        ("A", "Acceleration [g]", lambda r: np.array(r))
    ]

    fig2, axes2 = plt.subplots(len(cons_vars), 1, figsize=(figsize[0], figsize[1]*0.6), sharex=True)
    cmap2 = plt.get_cmap('tab10')
    for i, run in enumerate(runs):
        res = run.get("res")
        if res is None:
            continue
        t = res["t"]
        base_color = cmap2(i % cmap2.N)
        passed = bool(run.get('PASS_all', False))
        color = base_color if passed else (*base_color[:3], 0.45)
        lw = 1.8 if passed else 1.0
        label = f"EFPA={run['efpa_deg']}°, LDv_hi={run['LDv_hi']}, LDv_lo={run['LDv_lo']}"
        for ax, (var, ylabel, transform) in zip(axes2, cons_vars):
            if var not in res:
                continue
            y = transform(res[var])
            ax.plot(t, y, label=label, color=color, linewidth=lw)
            # place marker at maximum of this variable
            try:
                idx_max = int(np.nanargmax(y))
                ax.plot(t[idx_max], y[idx_max], marker='o', color=base_color, markersize=5,
                        markeredgecolor='k' if passed else None)
            except Exception:
                pass
            ax.set_ylabel(ylabel)
            ax.grid(True)

    axes2[-1].set_xlabel("Time [s]")
    if title:
        fig2.suptitle(title)
    # do not show legend (too many traces); reserve space for title
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    # add constraint limit lines if sim and sim.constraints provided
    if sim is not None and hasattr(sim, 'constraints'):
        c = sim.constraints
        # q_max (Pa)
        try:
            axes2[0].axhline(c.q_max, color='k', linestyle='--', linewidth=1)
            axes2[0].text(0.98, 0.9, f"q_max={c.q_max:.0f} Pa", transform=axes2[0].transAxes,
                          horizontalalignment='right', fontsize='small')
        except Exception:
            pass
        # Qdot_max (W/m^2)
        try:
            axes2[1].axhline(c.Qdot_max, color='k', linestyle='--', linewidth=1)
            axes2[1].text(0.98, 0.9, f"Qdot_max={c.Qdot_max:.0f}", transform=axes2[1].transAxes,
                          horizontalalignment='right', fontsize='small')
        except Exception:
            pass
        # A_max (g)
        try:
            axes2[2].axhline(c.A_max, color='k', linestyle='--', linewidth=1)
            axes2[2].text(0.98, 0.9, f"A_max={c.A_max:.2f} g", transform=axes2[2].transAxes,
                          horizontalalignment='right', fontsize='small')
        except Exception:
            pass

    fig2.savefig(constraints_file, dpi=150)
    print(f"Saved constraints figure: {constraints_file}")

    plt.show()
    return fig1, fig2


if __name__ == "__main__":
    mars = MarsConstants()
    veh = VehicleProperties()
    ic = InitialConditions()
    tgt = TargetConditions()
    cons = PathConstraints()

    sim = ReentrySimulation(mars, veh, ic, tgt, cons)

    # Run a bank-schedule sweep for fixed EFPA = -12 deg and compute reachable set
    df, runs = run_bank_schedule_reachable_set(
        sim,
        efpa_deg=-12.0,
        early_bank_deg=(45, 115, 50),
        early_V=(4500, 6000, 1000),
        late_bank_deg=(40, 60, 20),
        late_V=(2000, 3500, 1000),
        time_step=0.1, guidance_dt=0.5,
        out_csv="reachable_set.csv"
    )
    print(df.head())

    # Plot reachable set: CrossRange vs Downrange colored by final altitude
    title = "Reachable set (EFPA=-12\u00b0)"
    _ = plot_reachable_set(df, title=title, out_png="reachable_set.png")

