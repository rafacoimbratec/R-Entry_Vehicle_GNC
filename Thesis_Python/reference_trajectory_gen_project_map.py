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
    L_over_D: float = 0.54          # nominal L/D
    beta: float = 354.0             # [kg/m^2] ballistic coefficient
    # Heating correlation (monitoring only)
    N: float = 0.5
    M: float = 3.15
    k_heat_flux: float = 5.3697e-5  # SI -> W/m^2 with rho^N * V^M

@dataclass
class InitialConditions:
    h: float = 125e3
    theta: float = -176.40167 * np.pi/180
    phi: float = -21.3 * np.pi/180
    V: float = 4700.0
    gamma: float = -10.0 * np.pi/180
    psi: float = -2.8758 * np.pi/180

@dataclass
class TargetConditions:
    theta: float = -175.8 * np.pi/180
    phi: float = 0.276 * np.pi/180
    V: float = 450.0
    h: float = 2480.0
    Rgo_target: float = 5e3

@dataclass
class PathConstraints:
    A_max: float = 4.0          # [g] converted inside if needed
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
        res.update({"Mach": np.zeros(n), "downrange_m": np.zeros(n)})

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
            res["Mach"][i] = state.V / max(a, 1e-6)

            # stop at deploy (NEW RULE): V <= 450 m/s OR h <= 6000 m
            if (state.V <= 450.0) or (state.h <= 6000.0):
                #print(f"Deploy condition met at t={ti:.1f} s: V={state.V:.1f} m/s, h={state.h:.1f} m")
                for k in res:
                    res[k] = res[k][:i+1]
                break

            # integrate
            state = self.dyn.rk4_step(state, sigma, time_step)

        return res

# -----------------------
# Spherical metrics (downrange/crossrange/Rgo)
# -----------------------
def spherical_metrics(lat0, lon0, lat, lon, lat_t, lon_t, Rplanet):
    def clamp(x, lo=-1.0, hi=1.0): return np.clip(x, lo, hi)
    def sph2cart(lat_, lon_):
        return np.array([np.cos(lat_)*np.cos(lon_), np.cos(lat_)*np.sin(lon_), np.sin(lat_)])
    cos_d = np.sin(lat0)*np.sin(lat) + np.cos(lat0)*np.cos(lat)*np.cos(lon - lon0)
    cos_d = clamp(cos_d); d = np.arccos(cos_d); sin_d = np.sqrt(max(0.0, 1.0 - cos_d**2))
    rE, rP, rT = sph2cart(lat0, lon0), sph2cart(lat, lon), sph2cart(lat_t, lon_t)
    nOET = np.cross(rE, rT); nOET = nOET/np.linalg.norm(nOET)
    nOEP = np.cross(rE, rP); nrm = np.linalg.norm(nOEP)
    nOEP = nOET if nrm < 1e-12 else nOEP/nrm
    iang = np.arccos(clamp(np.dot(nOEP, nOET)))
    RC_ang = np.arcsin(clamp(np.sin(iang) * sin_d))
    cos_RC = np.cos(RC_ang); denom = np.sign(cos_RC)*np.maximum(np.abs(cos_RC), 1e-12)
    RD_ang = np.arccos(clamp(cos_d / denom))
    cos_dt = np.sin(lat0)*np.sin(lat_t) + np.cos(lat0)*np.cos(lat_t)*np.cos(lon_t - lon0)
    RD_tot_ang = np.arccos(clamp(cos_dt / denom))
    RD_go = (RD_tot_ang - RD_ang) * Rplanet
    Rgo   = np.sqrt(RD_go**2 + (RC_ang * Rplanet)**2)
    RC, RD = RC_ang * Rplanet, RD_ang * Rplanet
    return d*Rplanet, sin_d, cos_d, RC, RD, RD_go, Rgo

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

# -----------------------
# Design-map settings & sweep
# -----------------------
@dataclass
class DesignMapSettings:
    efpa_deg_grid: tuple = (-16.0, -15.0, 0.2)   # start, stop, step
    LDv_hi_grid:  tuple = (0.1, 0.2, 0.02)
    LDv_lo_grid:  tuple = (0.2, 0.3, 0.02)
    V_hi: float = 5500.0
    V_lo: float = 2000.0
    # Filters (Table 2 style)
    miss_km: float = 700.0
    sat_max_frac: float = 0.25
    mach_ref_min: float = 2.2
    alt_rate_min: float = 0.0
    deploy_alt_min_m: float = 600.0
    ranoff_km: float = 5.0  # tweak per mission

def _mach_from_state(atm, mars, h, V):
    a = math.sqrt(mars.specific_heat_ratio * mars.R_s * atm.temperature(h))
    return V / max(a, 1e-6)

def _evaluate_trajectory(sim: ReentrySimulation, gobj: BankScheduleGuidance, ic_override=None):
    # allow EFPA override
    if ic_override is not None:
        sim_local = ReentrySimulation(sim.mars, sim.veh, ic_override, sim.target, sim.constraints)
    else:
        sim_local = sim

    res = sim_local.run(time_step=0.1, guidance=gobj, guidance_dt=0.5)

    # --- NEW DEPLOY RULE: first time V <= 450 m/s OR h <= 6000 m
    V_arr, h_arr = res["V"], res["h"]
    hits = np.where((V_arr <= 450.0) | (h_arr <= 6000.0))[0]
    if hits.size > 0:
        idx = int(hits[0])
    else:
        # fallback to closest threshold
        i_v = int(np.argmin(np.abs(V_arr - 450.0)))
        i_h = int(np.argmin(np.abs(h_arr - 6000.0)))
        idx = i_v if (abs(V_arr[i_v] - 450.0) <= abs(h_arr[i_h] - 6000.0)) else i_h

    # metrics at deploy
    lat_dep, lon_dep = res["phi"][idx], res["theta"][idx]
    _, _, _, RC, RD, RD_go, Rgo = spherical_metrics(
        sim.ic.phi, sim.ic.theta, lat_dep, lon_dep, sim.target.phi, sim.target.theta, sim.mars.radius
    )
    miss_km = abs(Rgo)/1000.0
    range_flown_km = RD/1000.0
    alt_rate = res["V"][idx] * np.sin(res["gamma"][idx])    # dh/dt
    M = _mach_from_state(sim.atm, sim.mars, res["h"][idx], res["V"][idx])

    # saturation fraction: when LDv hits +/- LD_nom (acos arg -> ±1)
    LD_nom = sim.veh.L_over_D
    sigma = res["sigma"]
    LDv = LD_nom * np.cos(sigma)
    sat_frac = float(np.mean(np.isclose(np.abs(LDv), LD_nom, atol=1e-6)))

    out = {
        "idx": idx,
        "deploy_alt_m": res["h"][idx],
        "deploy_speed_mps": res["V"][idx],
        "deploy_mach": M,
        "miss_km": miss_km,
        "range_flown_km": range_flown_km,
        "alt_rate_ms": alt_rate,
        "sat_frac": sat_frac
    }
    print(f"Evaluated trajectory: deploy_mach={out['deploy_mach']:.2f}, deploy_alt_km={out['deploy_alt_m']*1e-3:.1f} km, miss={out['miss_km']:.2f} km, range_flown={out['range_flown_km']:.2f} km, alt_rate={out['alt_rate_ms']:.1f} m/s, sat_frac={out['sat_frac']:.3f}")
    return out, res

def design_map_sweep(sim: ReentrySimulation, s: DesignMapSettings) -> pd.DataFrame:
    efpas = np.arange(s.efpa_deg_grid[0], s.efpa_deg_grid[1] + 1e-9, s.efpa_deg_grid[2])
    LDv_hi_list = np.arange(s.LDv_hi_grid[0], s.LDv_hi_grid[1] + 1e-9, s.LDv_hi_grid[2])
    LDv_lo_list = np.arange(s.LDv_lo_grid[0], s.LDv_lo_grid[1] + 1e-9, s.LDv_lo_grid[2])

    rows = []
    # baseline entry->target downrange for "required range": baseline + ranoff + 5 km
    _, _, _, _, RD_base, _, _ = spherical_metrics(sim.ic.phi, sim.ic.theta, sim.ic.phi, sim.ic.theta,
                                                  sim.target.phi, sim.target.theta, sim.mars.radius)
    #required_range_km_base = RD_base/1000.0 + s.ranoff_km + 5.0

    for efpa_deg in efpas:
        ic_mod = InitialConditions(h=sim.ic.h, theta=sim.ic.theta, phi=sim.ic.phi,
                                   V=sim.ic.V, gamma=np.deg2rad(efpa_deg), psi=sim.ic.psi)

        for ldv_hi in LDv_hi_list:
            for ldv_lo in LDv_lo_list:
                if ldv_lo > ldv_hi:
                    continue
                g = BankScheduleGuidance(sim.veh.L_over_D, s.V_hi, s.V_lo, ldv_hi, ldv_lo)

                # reference run
                met_ref, res_ref = _evaluate_trajectory(sim, g, ic_override=ic_mod)

                # filters on reference
                ref_ok = ((met_ref["deploy_alt_m"])>= s.deploy_alt_min_m) and \
                         (met_ref["deploy_mach"] >= s.mach_ref_min) #and \
                         #(met_ref["range_flown_km"] >= required_range_km_base)

                # stress cases: EFPA ±0.2 deg for saturation max
                sat_max = met_ref["sat_frac"]
                for dEFPA in (-0.2, +0.2):
                    ic_stress = InitialConditions(h=sim.ic.h, theta=sim.ic.theta, phi=sim.ic.phi,
                                                  V=sim.ic.V, gamma=np.deg2rad(efpa_deg + dEFPA), psi=sim.ic.psi)
                    met_st, _ = _evaluate_trajectory(sim, g, ic_override=ic_stress)
                    sat_max = max(sat_max, met_st["sat_frac"])

                pass_all = ref_ok and \
                           (met_ref["miss_km"] <= s.miss_km) and \
                           (sat_max <= s.sat_max_frac) and \
                           ((met_ref["alt_rate_ms"]*-1) >= s.alt_rate_min)

                rows.append({
                    "efpa_deg": efpa_deg,
                    "LDv_hi": ldv_hi,
                    "LDv_lo": ldv_lo,
                    "deploy_alt_m": met_ref["deploy_alt_m"],
                    "deploy_mach": met_ref["deploy_mach"],
                    "miss_km": met_ref["miss_km"],
                    "range_flown_km": met_ref["range_flown_km"],
                    "alt_rate_ms": met_ref["alt_rate_ms"],
                    "sat_frac_max": sat_max,
                    #"required_range_km": required_range_km_base,
                    "PASS": bool(pass_all)
                })
                #print(bool(pass_all))
                #print("Candidate didn't meet because: " +
                      #("" if ref_ok else " Reference failed;") +
                      #("" if (met_ref["miss_km"] <= s.miss_km) else " Miss distance exceeded;") +
                      #("" if (sat_max <= s.sat_max_frac) else " Saturation fraction exceeded;") +
                      #("" if ((met_ref["alt_rate_ms"]*-1) >= s.alt_rate_min) else " Altitude rate exceeded;"))
                #print("" if pass_all else "Candidate PASSED all filters!")

    df = pd.DataFrame(rows)
    if (df["PASS"] == True).any():
        best = df[df["PASS"] == True].sort_values("deploy_alt_m", ascending=False).iloc[0]
        print("\nSelected reference (max deploy altitude among PASS):\n")
        print(best.to_string())
    else:
        print("\nNo candidate passed all filters. Consider adjusting grids or thresholds.\n")
    return df

# =======================
# Plotting for candidate
# =======================

def _deploy_index_from_rule(res):
    V_arr, h_arr = res["V"], res["h"]
    idxs = np.where((V_arr <= 450.0) | (h_arr <= 6000.0))[0]
    if idxs.size > 0:
        return int(idxs[0])
    # fallback to nearest threshold if never crossed
    i_v = int(np.argmin(np.abs(V_arr - 450.0)))
    i_h = int(np.argmin(np.abs(h_arr - 6000.0)))
    return i_v if (abs(V_arr[i_v] - 450.0) <= abs(h_arr[i_h] - 6000.0)) else i_h

def _downrange_from_entry(sim, res):
    # great-circle downrange from entry to current (meters)
    dr = np.zeros_like(res["t"])
    for i in range(len(res["t"])):
        _, _, _, _, RD, _, _ = spherical_metrics(
            sim.ic.phi, sim.ic.theta,
            res["phi"][i], res["theta"][i],
            sim.target.phi, sim.target.theta,
            sim.mars.radius
        )
        dr[i] = RD
    return dr

def plot_candidate_trajectory(sim, efpa_deg, LDv_hi, LDv_lo, V_hi=5500.0, V_lo=2000.0, save_csv=True):
    """Re-simulates the chosen candidate and produces plots."""
    ic_mod = InitialConditions(h=sim.ic.h, theta=sim.ic.theta, phi=sim.ic.phi,
                               V=sim.ic.V, gamma=np.deg2rad(efpa_deg), psi=sim.ic.psi)
    sim_local = ReentrySimulation(sim.mars, sim.veh, ic_mod, sim.target, sim.constraints)
    g = BankScheduleGuidance(sim.veh.L_over_D, V_hi, V_lo, LDv_hi, LDv_lo)

    res = sim_local.run(time_step=0.1, guidance=g, guidance_dt=0.5)
    idx_dep = _deploy_index_from_rule(res)
    t = res["t"]
    dr_km = _downrange_from_entry(sim_local, res) / 1000.0

    if save_csv:
        # Save trajectory for reference
        traj = pd.DataFrame({
            "t_s": t,
            "alt_m": res["h"],
            "downrange_km": dr_km,
            "V_mps": res["V"],
            "gamma_deg": np.degrees(res["gamma"]),
            "q_Pa": res["q"],
            "Qdot_Wm2": res["Qdot"],
            "sigma_deg": np.degrees(res["sigma"]),
            "Mach": res["Mach"],
            "lat_deg": np.degrees(res["phi"]),
            "lon_deg": np.degrees(res["theta"]),
        })
        traj.to_csv("selected_candidate_trajectory.csv", index=False)
        print("Saved: selected_candidate_trajectory.csv")

    # ------ Plots ------
    # 1) Altitude vs Downrange
    plt.figure()
    plt.plot(dr_km, res["h"]/1000.0)
    plt.plot(dr_km[idx_dep], res["h"][idx_dep]/1000.0, "o")
    plt.xlabel("Downrange [km]")
    plt.ylabel("Altitude [km]")
    plt.title("Altitude vs Downrange (selected candidate)")
    plt.grid(True)

    # 2) Speed vs Time
    plt.figure()
    plt.plot(t, res["V"])
    plt.plot(t[idx_dep], res["V"][idx_dep], "o")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed vs Time")
    plt.grid(True)

    # 3) Flight-path angle vs Time
    plt.figure()
    plt.plot(t, np.degrees(res["gamma"]))
    plt.plot(t[idx_dep], np.degrees(res["gamma"][idx_dep]), "o")
    plt.xlabel("Time [s]")
    plt.ylabel("FPA γ [deg]")
    plt.title("Flight-Path Angle vs Time")
    plt.grid(True)

    # 4) Dynamic pressure vs Time
    plt.figure()
    plt.plot(t, res["q"])
    plt.plot(t[idx_dep], res["q"][idx_dep], "o")
    plt.xlabel("Time [s]")
    plt.ylabel("Dynamic pressure q [Pa]")
    plt.title("Dynamic Pressure vs Time")
    plt.grid(True)

    # 5) Stagnation heat rate vs Time
    plt.figure()
    plt.plot(t, res["Qdot"])
    plt.plot(t[idx_dep], res["Qdot"][idx_dep], "o")
    plt.xlabel("Time [s]")
    plt.ylabel("Stagnation heat rate [W/m$^2$]")
    plt.title("Sutton–Graves Heat Rate vs Time")
    plt.grid(True)

    # 6) Bank angle vs Time
    plt.figure()
    plt.plot(t, np.degrees(res["sigma"]))
    plt.plot(t[idx_dep], np.degrees(res["sigma"][idx_dep]), "o")
    plt.xlabel("Time [s]")
    plt.ylabel("Bank angle σ [deg]")
    plt.title("Bank Angle vs Time")
    plt.grid(True)

    # 7) Mach vs Time
    plt.figure()
    plt.plot(t, res["Mach"])
    plt.plot(t[idx_dep], res["Mach"][idx_dep], "o")
    plt.xlabel("Time [s]")
    plt.ylabel("Mach [-]")
    plt.title("Mach vs Time")
    plt.grid(True)

    plt.show()
    return res

def plot_best_from_dataframe(sim, df, settings):
    """Pick the best PASS from df and plot it."""
    df_pass = df[df["PASS"] == True]
    if df_pass.empty:
        print("No PASS candidates to plot.")
        return None
    best = df_pass.sort_values("deploy_alt_m", ascending=False).iloc[0]
    print("\nPlotting selected candidate:")
    print(best.to_string())
    return plot_candidate_trajectory(sim,
                                     efpa_deg=best["efpa_deg"],
                                     LDv_hi=best["LDv_hi"],
                                     LDv_lo=best["LDv_lo"],
                                     V_hi=settings.V_hi,
                                     V_lo=settings.V_lo)


# -----------------------
# Run sweep
# -----------------------
if __name__ == "__main__":
    mars = MarsConstants()
    veh = VehicleProperties()
    ic = InitialConditions()
    tgt = TargetConditions()
    cons = PathConstraints()

    sim = ReentrySimulation(mars, veh, ic, tgt, cons)

    settings = DesignMapSettings(
        efpa_deg_grid=(-16.0, -15.0, 0.2),
        LDv_hi_grid=(0.1, 0.2, 0.02),
        LDv_lo_grid=(0.2, 0.3, 0.02),
        V_hi=5500.0,
        V_lo=2000.0,
        miss_km=700.0,
        sat_max_frac=0.25,
        mach_ref_min=2.2,
        alt_rate_min=0.0,
        deploy_alt_min_m=600.0,
        ranoff_km=5.0
    )

    df = design_map_sweep(sim, settings)
    df.to_csv("design_map_results.csv", index=False)
    print("\nSaved: design_map_results.csv")
    _ = plot_best_from_dataframe(sim, df, settings)

