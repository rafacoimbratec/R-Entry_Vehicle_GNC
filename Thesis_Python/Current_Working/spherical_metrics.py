import numpy as np

def spherical_metrics(lat0, lon0, lat, lon, lat_t, lon_t, Rplanet):
    """
    Computes spherical distance, cross-range, and down-range metrics.

    Inputs (all angles in radians):
        lat0, lon0   - reference (entry) latitude and longitude
        lat,  lon    - current latitude and longitude
        lat_t, lon_t - target latitude and longitude
        Rplanet      - planetary radius [m]

    Outputs:
        d     - great-circle distance between reference and current point [m]
        sin_d - sine of central angle
        cos_d - cosine of central angle
        RC    - cross-range distance [m]
        RD    - down-range distance [m]
        RD_go - remaining downrange distance to target [m]
        Rgo   - total range-to-go (hypotenuse of downrange and crossrange) [m]
    """

    # --- Helpers ---
    def clamp(x, lo=-1.0, hi=1.0):
        return np.clip(x, lo, hi)

    def norm(v):
        return np.linalg.norm(v, axis=-1, keepdims=False)

    def safe_unit(v, eps=1e-12):
        n = norm(v)
        # Avoid division by ~0; where n<eps, return zero vector
        with np.errstate(divide='ignore', invalid='ignore'):
            out = np.where(n[..., None] > eps, v / n[..., None], 0.0)
        return out, n

    def sph2cartvec(lat_, lon_):
        x = np.cos(lat_) * np.cos(lon_)
        y = np.cos(lat_) * np.sin(lon_)
        z = np.sin(lat_)
        return np.stack((x, y, z), axis=-1)

    # --- Central angle between reference (lat0, lon0) and current (lat, lon) ---
    cos_d = np.sin(lat0)*np.sin(lat) + np.cos(lat0)*np.cos(lat)*np.cos(lon - lon0)
    cos_d = clamp(cos_d, -1.0, 1.0)
    d = np.arccos(cos_d)                        # [rad]
    sin_d = np.sqrt(np.maximum(0.0, 1.0 - cos_d**2))

    # --- Convert spherical (lat, lon) to Cartesian (unit sphere) ---
    rE = sph2cartvec(lat0, lon0)               # reference
    rP = sph2cartvec(lat,  lon)                 # current
    rT = sph2cartvec(lat_t, lon_t)              # target

    # --- Normals to trajectory and target planes ---
    nOET = np.cross(rE, rT)
    nOET, _ = safe_unit(nOET)

    nOEP = np.cross(rE, rP)
    nOEP_unit, nOEP_norm = safe_unit(nOEP)

    # Special case: entry point == current point -> use target plane normal
    use_target_plane = (np.ndim(nOEP_norm) == 0 and nOEP_norm < 1e-12) or \
                       (np.ndim(nOEP_norm) > 0 and np.any(nOEP_norm < 1e-12))

    # If scalar, easy; if array, blend elementwise
    if np.isscalar(nOEP_norm):
        if nOEP_norm < 1e-12:
            nOEP_unit = nOET
    else:
        nOEP_unit = np.where((nOEP_norm[..., None] < 1e-12), nOET, nOEP_unit)

    # --- Angle between trajectory plane and target plane ---
    dot_planes = np.sum(nOEP_unit * nOET, axis=-1)
    dot_planes = clamp(dot_planes, -1.0, 1.0)
    i = np.arccos(dot_planes)

    # --- Cross-range angle ---
    RC_ang = np.arcsin(clamp(np.sin(i) * sin_d, -1.0, 1.0))

    # --- Sign of cross-range ---
    cross_planes = np.cross(nOEP_unit, nOET)
    cross_norm = norm(cross_planes)
    # Avoid division by ~0
    with np.errstate(divide='ignore', invalid='ignore'):
        cross_hat = np.where(cross_norm[..., None] > 1e-12,
                             cross_planes / cross_norm[..., None],
                             0.0)  # arbitrary, sign won't matter if zero
    sign_val = np.sum(cross_hat * rE, axis=-1)
    RC_ang = np.where(sign_val < 0, -RC_ang, RC_ang)

    # --- Down-range angle from entry to current ---
    # RD = acos(cos_d / cos(RC))
    cos_RC = np.cos(RC_ang)
    denom = np.where(np.abs(cos_RC) < 1e-12, np.sign(cos_RC) * 1e-12, cos_RC)
    val = clamp(cos_d / denom, -1.0, 1.0)
    RD_ang = np.arccos(val)

    # --- Total down-range (entry to target) ---
    cos_dt = np.sin(lat0)*np.sin(lat_t) + np.cos(lat0)*np.cos(lat_t)*np.cos(lon_t - lon0)
    cos_dt = clamp(cos_dt, -1.0, 1.0)
    # dt = np.arccos(cos_dt)  # (not needed below)
    val_tot = clamp(cos_dt / denom, -1.0, 1.0)
    RD_tot_ang = np.arccos(val_tot)

    # --- Remaining downrange and total range-to-go (meters) ---
    RD_go = (RD_tot_ang - RD_ang) * Rplanet
    Rgo   = np.sqrt(RD_go**2 + (RC_ang * Rplanet)**2)

    # --- Convert angular measures to distances ---
    RC = RC_ang * Rplanet
    RD = RD_ang * Rplanet
    d_m = d * Rplanet

    return d_m, sin_d, cos_d, RC, RD, RD_go, Rgo


# -------- Example usage (three sanity cases with error printouts) --------
if __name__ == "__main__":
    deg2rad = np.pi / 180.0
    R = 3396.2e3  # Mars mean radius [m]

    # Entry (reference) and target (same as your example)
    lat0  = -21.3 * deg2rad
    lon0  = -176.40167 * deg2rad
    lat_t =   0.276 * deg2rad
    lon_t = -175.8    * deg2rad

    def sph2cart(lat, lon):
        return np.array([np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)])

    def cart2sph(v):
        x, y, z = v
        lat = np.arctan2(z, np.hypot(x, y))
        lon = np.arctan2(y, x)
        return lat, lon

    # Precompute entry->target arc and vectors
    rE, rT = sph2cart(lat0, lon0), sph2cart(lat_t, lon_t)
    arc_ET = np.arccos(np.clip(np.dot(rE, rT), -1.0, 1.0)) * R  # expected RD_go in Case 1

    # ========= Case 1: Same point (lat=lat0, lon=lon0) =========
    print("=== Case 1: Same point (lat=lat0, lon=lon0) ===")
    lat_curr = lat0
    lon_curr = lon0
    d, sin_d, cos_d, RC, RD, RD_go, Rgo = spherical_metrics(
        lat0, lon0, lat_curr, lon_curr, lat_t, lon_t, R
    )
    # expected
    exp_d = 0.0
    exp_RC = 0.0
    exp_RD = 0.0
    exp_RD_go = arc_ET
    exp_Rgo = exp_RD_go

    print("Everything")
    print(f"d     = {d:.6f} m    | err_abs = {abs(d-exp_d):.6e}")
    print(f"sin_d = {sin_d:.6f}")
    print(f"cos_d = {cos_d:.6f}")
    print(f"RC    = {RC:.6f} m  | err_abs = {abs(RC-exp_RC):.6e}")
    print(f"RD    = {RD:.6f} m  | err_abs = {abs(RD-exp_RD):.6e}")
    print(f"RD_go = {RD_go:.6f} m | err_abs = {abs(RD_go-exp_RD_go):.6e}")
    print(f"Rgo   = {Rgo:.6f} m  | err_abs = {abs(Rgo-exp_Rgo):.6e}")
    print("Done\n")

    # ========= Case 2: On target great-circle (RC≈0, RD≈d) =========
    print("=== Case 2: On target great-circle (RC≈0, RD≈d) ===")
    # Slerp 1/3 of the way from entry to target along their great circle
    cos_dtot = np.clip(np.dot(rE, rT), -1.0, 1.0)
    dtot = np.arccos(cos_dtot)
    f = 1.0/3.0
    denom = np.sin(dtot) if dtot > 1e-12 else 1.0
    v = (np.sin((1-f)*dtot)*rE + np.sin(f*dtot)*rT) / denom
    lat_curr, lon_curr = cart2sph(v)
    print(lat_curr / deg2rad, lon_curr / deg2rad)  # debug

    d2, _, _, RC2, RD2, RD_go2, Rgo2 = spherical_metrics(
        lat0, lon0, lat_curr, lon_curr, lat_t, lon_t, R
    )
    # expected (within numerical tolerance)
    exp_d2 = f * dtot * R
    exp_RC2 = 0.0
    exp_RD2 = exp_d2
    exp_RD_go2 = (1.0 - f) * dtot * R
    exp_Rgo2 = exp_RD_go2

    def rel_err(x, xexp):
        return (abs(x - xexp) / max(abs(xexp), 1e-12))

    print("Everything")
    print(f"d     = {d2:.6f} m    | err_abs = {abs(d2-exp_d2):.6e}, err_rel = {rel_err(d2,exp_d2):.3e}")
    print(f"RC    = {RC2:.6f} m  | err_abs = {abs(RC2-exp_RC2):.6e}")
    print(f"RD    = {RD2:.6f} m  | err_abs = {abs(RD2-exp_RD2):.6e}, err_rel = {rel_err(RD2,exp_RD2):.3e}")
    print(f"RD_go = {RD_go2:.6f} m | err_abs = {abs(RD_go2-exp_RD_go2):.6e}, err_rel = {rel_err(RD_go2,exp_RD_go2):.3e}")
    print(f"Rgo   = {Rgo2:.6f} m  | err_abs = {abs(Rgo2-exp_Rgo2):.6e}, err_rel = {rel_err(Rgo2,exp_Rgo2):.3e}")
    print("Done\n")

    # ========= Case 3: Near 90° cross-range (stress denom, stays finite) =========
    print("=== Case 3: Near 90° cross-range (finite results) ===")
    # Construct rP so that plane angle i≈90° and central angle d≈90°
    nOET = np.cross(rE, rT); nOET = nOET / np.linalg.norm(nOET)
    m = np.cross(nOET, rE); m = m / np.linalg.norm(m)     # m ⟂ rE and ⟂ nOET
    rP = np.cross(m, rE);   rP = rP / np.linalg.norm(rP)  # rE·rP = 0 ⇒ d ≈ 90°
    lat_curr, lon_curr = cart2sph(rP)

    d3, _, _, RC3, RD3, RD_go3, Rgo3 = spherical_metrics(
        lat0, lon0, lat_curr, lon_curr, lat_t, lon_t, R
    )

    # expected (approximate, due to epsilon guard): d ≈ π/2 * R, RC ≈ π/2 * R, RD ≈ π/2 * R
    exp_d3 = 0.5 * np.pi * R
    exp_RC3 = 0.5 * np.pi * R
    exp_RD3 = 0.5 * np.pi * R

    print("Everything")
    print(f"d     = {d3:.6f} m    | err_abs ≈ {abs(d3-exp_d3):.6e}, err_rel ≈ {rel_err(d3,exp_d3):.3e}")
    print(f"RC    = {RC3:.6f} m  | err_abs ≈ {abs(RC3-exp_RC3):.6e}, err_rel ≈ {rel_err(RC3,exp_RC3):.3e}")
    print(f"RD    = {RD3:.6f} m  | err_abs ≈ {abs(RD3-exp_RD3):.6e}, err_rel ≈ {rel_err(RD3,exp_RD3):.3e}")
    print(f"RD_go = {RD_go3:.6f} m (finite)")
    print(f"Rgo   = {Rgo3:.6f} m  (finite)")
    print("Done")
