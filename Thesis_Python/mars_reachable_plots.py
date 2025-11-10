import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

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
    T_const = 210.0            # [K]

    @property
    def a_sound(self):
        Rspec = MarsEnv.R_star / MarsEnv.M_CO2
        return np.sqrt(MarsEnv.gamma_gas * Rspec * MarsEnv.T_const)


class Vehicle:
    L_over_D = 0.24
    beta = 135.0
    N = 0.5
    M = 3.0
    k_heat_flux = 5.3697e-5


mars = MarsEnv()
veh = Vehicle()

# ============================================================
# spherical_metrics (from user)
# ============================================================
def spherical_metrics(lat0, lon0, lat, lon, lat_t, lon_t, Rplanet):
    def clamp(x, lo=-1.0, hi=1.0): return np.clip(x, lo, hi)
    def norm(v): return np.linalg.norm(v, axis=-1, keepdims=False)
    def safe_unit(v, eps=1e-12):
        n = norm(v)
        with np.errstate(divide='ignore', invalid='ignore'):
            out = np.where(n[..., None] > eps, v / n[..., None], 0.0)
        return out, n
    def sph2cartvec(lat_, lon_):
        x = np.cos(lat_) * np.cos(lon_)
        y = np.cos(lat_) * np.sin(lon_)
        z = np.sin(lat_)
        return np.stack((x, y, z), axis=-1)

    cos_d = np.sin(lat0)*np.sin(lat) + np.cos(lat0)*np.cos(lat)*np.cos(lon - lon0)
    cos_d = clamp(cos_d, -1.0, 1.0)
    d = np.arccos(cos_d)
    sin_d = np.sqrt(np.maximum(0.0, 1.0 - cos_d**2))

    rE = sph2cartvec(lat0, lon0)
    rP = sph2cartvec(lat, lon)
    rT = sph2cartvec(lat_t, lon_t)
    nOET = np.cross(rE, rT); nOET, _ = safe_unit(nOET)
    nOEP = np.cross(rE, rP); nOEP_unit, nOEP_norm = safe_unit(nOEP)
    if np.isscalar(nOEP_norm):
        if nOEP_norm < 1e-12: nOEP_unit = nOET
    else:
        nOEP_unit = np.where((nOEP_norm[..., None] < 1e-12), nOET, nOEP_unit)
    dot_planes = np.sum(nOEP_unit * nOET, axis=-1)
    dot_planes = clamp(dot_planes, -1.0, 1.0)
    i = np.arccos(dot_planes)
    RC_ang = np.arcsin(clamp(np.sin(i) * sin_d, -1.0, 1.0))
    cross_planes = np.cross(nOEP_unit, nOET); cross_norm = norm(cross_planes)
    with np.errstate(divide='ignore', invalid='ignore'):
        cross_hat = np.where(cross_norm[..., None] > 1e-12,
                             cross_planes / cross_norm[..., None], 0.0)
    sign_val = np.sum(cross_hat * rE, axis=-1)
    RC_ang = np.where(sign_val < 0, -RC_ang, RC_ang)
    cos_RC = np.cos(RC_ang)
    denom = np.where(np.abs(cos_RC) < 1e-12, np.sign(cos_RC)*1e-12, cos_RC)
    val = clamp(cos_d / denom, -1.0, 1.0)
    RD_ang = np.arccos(val)
    cos_dt = np.sin(lat0)*np.sin(lat_t) + np.cos(lat0)*np.cos(lat_t)*np.cos(lon_t - lon0)
    cos_dt = clamp(cos_dt, -1.0, 1.0)
    val_tot = clamp(cos_dt / denom, -1.0, 1.0)
    RD_tot_ang = np.arccos(val_tot)
    RD_go = (RD_tot_ang - RD_ang) * Rplanet
    Rgo = np.sqrt(RD_go**2 + (RC_ang * Rplanet)**2)
    RC = RC_ang * Rplanet
    RD = RD_ang * Rplanet
    d_m = d * Rplanet
    return d_m, sin_d, cos_d, RC, RD, RD_go, Rgo

# ============================================================
# Equations of motion
# ============================================================
def eom(state, sigma):
    r, th, ph, V, gam, psi = state
    g = mars.mu / r**2
    rho = mars.rho0 * np.exp(-(r - mars.radius) / mars.Hs)
    D = 0.5 * rho * V**2 / veh.beta
    L = veh.L_over_D * D
    Vh = V * np.cos(gam)

    r_dot = V * np.sin(gam)
    th_dot = (Vh * np.sin(psi)) / (r * np.cos(ph))
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
def bank_profile(t, sigma_const=np.deg2rad(180.0), t1=45.0, t2=100.0):
    if t < t1:
        return sigma_const
    elif t < t2:
        # linear ramp to zero
        return sigma_const * (1 - (t - t1)/(t2 - t1))
    else:
        return 0.0

# ============================================================
# Integrate one trajectory until deployment trigger
# ============================================================
def simulate_entry(initial_state, dt=0.25, tmax=2000.0):
    state = initial_state.copy()
    a_sound = mars.a_sound
    t = 0.0
    while t < tmax:
        sigma = bank_profile(t)
        state = rk4_step(eom, state, sigma, dt)
        r, th, ph, V, gam, psi = state
        h = r - mars.radius
        rho = mars.rho0 * np.exp(-h / mars.Hs)
        q = 0.5 * rho * V**2
        mach = V / a_sound
        if ((1.4 <= mach <= 2.2) and (300 <= q <= 800)) or (h <= 0):
            break
        t += dt
    return state, t


def simulate_entry_with_trace(initial_state, dt=0.25, tmax=2000.0):
    """Simulate entry while recording state history and sigma(t).

    Returns: times (1D), states (N x 6), sigmas (1D), trigger_index (int)
    """
    state = initial_state.copy()
    a_sound = mars.a_sound
    t = 0.0
    times = [t]
    states = [state.copy()]
    sigmas = [bank_profile(t)]
    trigger_index = None
    while t < tmax:
        sigma = bank_profile(t)
        state = rk4_step(eom, state, sigma, dt)
        t += dt
        times.append(t)
        states.append(state.copy())
        sigmas.append(sigma)
        r, th, ph, V, gam, psi = state
        h = r - mars.radius
        rho = mars.rho0 * np.exp(-h / mars.Hs)
        q = 0.5 * rho * V**2
        mach = V / a_sound
        if ((1.4 <= mach <= 2.2) and (300 <= q <= 800)) or (h <= 0):
            trigger_index = len(times) - 1
            break

    return np.array(times), np.array(states), np.array(sigmas), trigger_index

# ============================================================
# Build the contour set by sweeping target lat/lon grid
# ============================================================
def build_reachable_set(lat0, lon0, lat_grid, lon_grid):
    results = []
    # Fixed entry conditions
    h0 = 120e3
    r0 = mars.radius + h0
    V0 = 4700.0
    gam0 = np.deg2rad(-15.0)
    psi0 = np.deg2rad(-2.8758)
    for lat_t in lat_grid:
        for lon_t in lon_grid:
            state0 = np.array([r0, lon0, lat0, V0, gam0, psi0])
            state_f, tf = simulate_entry(state0, tmax=4000.0)
            r, th, ph, V, gam, psi = state_f
            h = r - mars.radius
            # Compute metrics relative to the target
            d_m, sin_d, cos_d, RC, RD, RD_go, Rgo = spherical_metrics(
                lat0, lon0, ph, th, lat_t, lon_t, mars.radius)
            results.append((RD/1000.0, RC/1000.0, h/1000.0))
    return np.array(results)

# ============================================================
# Define sweep grid and compute reachable set
# ============================================================
deg2rad = np.pi/180
lat0 = -21.3 * deg2rad
lon0 = -176.40167 * deg2rad

# sweep ±3° around the nominal entry projection
# sweep ±15° latitude, ±20° longitude
lat_grid = np.linspace(lat0 - 15*deg2rad, lat0 + 15*deg2rad, 10)
lon_grid = np.linspace(lon0 - 20*deg2rad, lon0 + 20*deg2rad, 10)


# ------------------------------------------------------------
# Run one example trajectory (trace sigma and states) and plot
# ------------------------------------------------------------
# choose a sample target a few degrees away from the entry projection
lat_t_example = lat0 + 5.0 * deg2rad
lon_t_example = lon0 + 10.0 * deg2rad

# initial state ordering: r, th(lon), ph(lat), V, gam, psi
state0_example = np.array([mars.radius + 120e3,
                           lon0,
                           lat0,
                           4700.0,
                           np.deg2rad(-15.0),
                           np.deg2rad(-2.8758)])

times, states, sigmas, trigger_idx = simulate_entry_with_trace(state0_example, dt=0.25, tmax=4000.0)
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
plt.scatter(np.degrees(state_trigger[1]), np.degrees(state_trigger[2]), color='orange', s=80, marker='*', label='deploy trigger')
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
# Prepare arrays for plotting
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

axes[6].plot(times, psi_array, '-y')
axes[6].set_ylabel('Heading ψ [deg]')

axes[7].plot(times, sigma_deg, color='0.5')
axes[7].axvline(t_trigger, color='orange', linestyle='--')
axes[7].set_ylabel('Bank σ [deg]')

for ax in axes:
    ax.set_xlabel('Time [s]')
    ax.grid(True)

plt.suptitle('All example trajectory states vs time', y=1.02)
plt.tight_layout()


res = build_reachable_set(lat0, lon0, lat_grid, lon_grid)
RD, RC, ALT = res[:,0], res[:,1], res[:,2]

# ============================================================
# Interpolate and contour plot (safe version)
# ============================================================
xi = np.linspace(RD.min(), RD.max(), 10)
yi = np.linspace(RC.min(), RC.max(), 10)
XI, YI = np.meshgrid(xi, yi)
ZI = griddata((RD, RC), ALT, (XI, YI), method='linear')

# Fill any NaNs (so we have a continuous surface)
if np.any(np.isnan(ZI)):
    ZI[np.isnan(ZI)] = np.nanmean(ALT)

# Ensure levels are strictly increasing
zmin, zmax = np.nanmin(ZI), np.nanmax(ZI)
if np.isclose(zmin, zmax):
    zmax = zmin + 1e-6  # avoid zero range
levels = np.linspace(zmin, zmax, 10)

plt.figure(figsize=(8,6))
cf = plt.contourf(XI, YI, ZI, levels=levels, cmap='viridis', extend='both')
cs = plt.contour(XI, YI, ZI, levels=levels, colors='k', linewidths=0.5)
plt.clabel(cs, fmt="%.1f", fontsize=8)
plt.colorbar(cf, label='Deployment altitude [km]')
plt.xlabel('Downrange [km]')
plt.ylabel('Crossrange [km]')
plt.title('Reachable-set contours (constant-then-ramp σ(t))')
plt.grid(True)
plt.tight_layout()
plt.show()





