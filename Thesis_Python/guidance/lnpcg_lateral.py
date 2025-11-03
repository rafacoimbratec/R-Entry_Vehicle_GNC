"""
LNPCG (Linear Numerical Predictor-Corrector Guidance) for Mars entry

Based on the energy-domain predictor-corrector method that:
1. Propagates trajectory forward using energy as independent variable
2. Uses Newton-Raphson to find initial bank angle that hits target range
3. Commands linear bank profile between current and final energy
"""

import numpy as np
from .base_guidance import BaseGuidance


class LNPCGuidanceLateral(BaseGuidance):
    """
    Linear Numerical Predictor-Corrector Guidance
    
    This guidance law:
    - Uses energy (e = μ/r - V²/2) as the independent variable
    - Propagates trajectory to predict landing range
    - Iteratively solves for bank angle that achieves target range
    - Commands linear bank profile from current to final energy
    """
    
    def __init__(self, 
                 sigma_f_deg: float = 0.0,
                 e_f: float = 12500221.74785505,
                 activation_time: float = 170.0,
                 max_iter: int = 50,
                 epsilon: float = 100.0,
                 de: float = 10000.0,
                 dsigma_deg: float = 3.0,
                 K_reasonable: float = 1.5):
        """
        Initialize LNPCG guidance
        
        Args:
            sigma_f_deg: Final bank angle at terminal energy [degrees]
            e_f: Terminal energy [J/kg]
            activation_time: Time to activate guidance [s]
            max_iter: Maximum Newton-Raphson iterations
            epsilon: Convergence tolerance for range error [m]
            de: Energy step for propagation [J/kg]
            dsigma_deg: Bank angle step for finite difference [degrees]
        """
        super().__init__(name="LNPCG")
        
        self.sigma_f = np.deg2rad(sigma_f_deg)
        self.e_f = e_f
        self.activation_time = activation_time
        self.max_iter = min(max_iter, 25)
        self.epsilon = epsilon
        self.de = de
        self.dsigma = np.deg2rad(dsigma_deg)
        
        # Initial guess for bank angle (will be updated during flight)
        self.sigma0_guess = np.deg2rad(100.0)
        self.sigma_prev = 0.0
        
        # Lateral predictive switching state
        self.n_reversals_allowed = 4  # default max reversals
        self.n_reversals_remain = self.n_reversals_allowed
        self.hysteresis = 0.05  # 5% hysteresis
        self.k_min = 0.5
        self.k_max = 5.0
        self.min_dwell = 5.0  # seconds between reversals
        self.last_reverse_time = -1e9
        self.last_update_time = -1e9
        # Max bank rate [rad/s]
        self.max_bank_rate = np.deg2rad(10.0)  # 10 deg/s default
        self._eps_small = 1e-6
        # Autonomy init flag and reasonable K for n0 calc
        self.K_reasonable = max(1.01, float(K_reasonable))
        self._auton_initialized = False
      
    def compute_bank_angle(self, t: float, state, target, mars_radius: float, **kwargs) -> float:
        """
        Compute LNPCG commanded bank angle
        
        Args:
            t: Current time [s]
            state: Current vehicle state
            target: Target conditions
            mars_radius: Mars radius [m]
            **kwargs: Must include 'atmosphere' and 'constraints'
        
        Returns:
            sigma: Commanded bank angle [rad]
        """
        # Before activation, use zero bank
        if t <= self.activation_time:
            #print(f"LNPCG inactive at t={t:.1f}s, using zero bank")
            return 0.0
        initial_phi = np.deg2rad(-21.3)
        initial_theta = np.deg2rad(-176.40167)
        
        # Get required objects from kwargs
        atmosphere = kwargs.get('atmosphere')
        constraints = kwargs.get('constraints')
        
        if atmosphere is None:
            raise ValueError("LNPCG requires 'atmosphere' in kwargs")
        
        # Compute current energy
        mu = 4.282837e13  # Mars gravitational parameter
        e_current = mu / state.r - 0.5 * state.V**2
        
        # Compute range-to-go using spherical metrics
        from spherical_metrics import spherical_metrics
        d, sin_d, cos_d, RC, RD, RD_go, Rgo = spherical_metrics(
           initial_phi, initial_theta, state.phi, state.theta, 
            target.phi, target.theta, mars_radius)
        
        s_target = Rgo  # Total range-to-go [m]
        #print(f"LNPCG at t={t:.1f}s: e_current={e_current:.1f} J/kg, s_target={s_target:.1f} m", f"sigma={np.rad2deg(self.sigma_prev):.2f}°")

        if abs(e_current - self.e_f) <= 1.0:
            return self.sigma_f
        
        # Adaptive bracketing: start centered around previous command
        #bracket_width = np.deg2rad(120)  # ±60° initial bracket
        sigma_low = np.deg2rad(0)
        sigma_high = np.deg2rad(200)
        
        # Compute function values at bracket bounds
        z_low = self._propagate_energy(state, sigma_low, atmosphere, 
                                       mu, mars_radius, e_current) - s_target
        z_high = self._propagate_energy(state, sigma_high, atmosphere, 
                                        mu, mars_radius, e_current) - s_target
        
        # If not bracketed, try widening once
        if z_low * z_high > 0:
            bracket_width = np.deg2rad(90)  # Widen to ±90°
            sigma_low = self.sigma_prev - bracket_width
            sigma_high = self.sigma_prev + bracket_width
            z_low = self._propagate_energy(state, sigma_low, atmosphere, 
                                           mu, mars_radius, e_current) - s_target
            z_high = self._propagate_energy(state, sigma_high, atmosphere, 
                                            mu, mars_radius, e_current) - s_target
        
        #print(f"  Bracketing: z({np.rad2deg(sigma_low):.1f}°)={z_low:.1f} m, z({np.rad2deg(sigma_high):.1f}°)={z_high:.1f} m")
        
        if z_low * z_high > 0:
            # Root still not bracketed - use previous bank angle
            print("Warning: LNPCG root not bracketed after widening, using previous bank angle")
            return self.sigma_prev
        
        # Use midpoint of bracket as initial guess
        sigma0_current = (sigma_low + sigma_high) / 2.0

        for k in range(self.max_iter):
            # Predict surface distance at current sigma0
            s_pred = self._propagate_energy(state, sigma0_current, atmosphere, 
                                           mu, mars_radius, e_current)
            
            # Convergence check
            if abs(s_pred - s_target) < self.epsilon:
                break
            
            # Centered finite difference for derivative
            s_pred_plus = self._propagate_energy(
                state, sigma0_current + self.dsigma, atmosphere, mu, mars_radius, e_current)
            s_pred_minus = self._propagate_energy(
                state, sigma0_current - self.dsigma, atmosphere, mu, mars_radius, e_current)
            
            dz_dsigma = (s_pred_plus - s_pred_minus) / (2 * self.dsigma)
            
            if abs(dz_dsigma) < 1e-10:
                # Derivative too small, can't update
                print("Warning: LNPCG derivative too small, stopping iteration")
                break
            
            # Newton-Raphson update
            sigma0_current = sigma0_current - (s_pred - s_target) / dz_dsigma
            #print(f"  Iter {k+1}: sigma0={np.rad2deg(sigma0_current):.2f}°, s_pred={s_pred:.1f} m, error={s_pred - s_target:.1f} m")
        
        # Update guess for next iteration
        self.sigma0_guess = sigma0_current

        # Compute nominal (unsigned) bank magnitude using the longitudinal solver
        sigma_mag = abs(sigma0_current)

        # ---- Lateral predictive switching logic ----
        # Compute current cross-range (chi_current) from entry to current state
        from spherical_metrics import spherical_metrics
        initial_phi = np.deg2rad(-21.3)
        initial_theta = np.deg2rad(-176.40167)
        d, sin_d, cos_d, RC_curr, RD_curr, RD_go_curr, Rgo_curr = spherical_metrics(
            initial_phi, initial_theta, state.phi, state.theta, target.phi, target.theta, mars_radius)
        chi_current = RC_curr  # meters (signed magnitude returned by spherical_metrics)

        # Initialize autonomous n0 based on a reasonable K (if not done yet)
        if not getattr(self, '_auton_initialized', False):
            # Use a predictor with a representative signed bank to estimate initial terminal cross-range
            sign_guess = 1.0 if abs(self.sigma_prev) < self._eps_small else np.sign(self.sigma_prev)
            try:
                s0, theta0, phi0 = self._propagate_energy(state, sign_guess * sigma_mag,
                                                          atmosphere, mu, mars_radius, e_current,
                                                          constant_bank=True, return_final=True)
                _, _, _, RC0, _, _, _ = spherical_metrics(initial_phi, initial_theta, phi0, theta0,
                                                        target.phi, target.theta, mars_radius)
                chi0 = abs(RC0)
            except Exception:
                # Fallback: use current cross-range as estimate
                chi0 = abs(chi_current)

            chi_f_init = getattr(target, 'Rgo_target', 5e3)
            if chi0 <= chi_f_init or self.K_reasonable <= 1.0:
                n0 = 0
            else:
                ratio0 = chi0 / max(chi_f_init, 1e-9)
                n0 = int(np.ceil(np.log(ratio0) / np.log(self.K_reasonable)))
                n0 = max(0, n0)

            # Set allowed and remaining reversals
            self.n_reversals_allowed = n0
            self.n_reversals_remain = n0
            self._auton_initialized = True
            # Optional: print diagnostic
            #print(f"Auton init: chi0={chi0:.1f} m, chi_f={chi_f_init:.1f} m, K={self.K_reasonable:.2f} -> n0={n0}")

        # Terminal cross-range tolerance
        chi_f = 0.01*1000.0  # 1% of 1 km = 10 m default

        # If within tolerance or no reversals left, keep current sign
        if self.n_reversals_remain <= 0 or abs(chi_current) < chi_f:
            desired_sign = np.sign(self.sigma_prev) if abs(self.sigma_prev) > self._eps_small else 1.0
        else:
            # Run longitudinal predictor twice with constant bank sign (no modeled reversals)
            # Predictor returns final lat/lon when constant_bank=True and return_final=True
            # Predict with current sign (+)
            sign_current = 1.0 if np.sign(self.sigma_prev) >= 0 else -1.0
            if abs(self.sigma_prev) < self._eps_small:
                # If previous sign is zero, assume +1
                sign_current = 1.0

            # Predict terminal cross-range for same sign and opposite sign
            s_pos, theta_pos, phi_pos = self._propagate_energy(state, sign_current * sigma_mag,
                                                               atmosphere, mu, mars_radius, e_current,
                                                               constant_bank=True, return_final=True)
            s_neg, theta_neg, phi_neg = self._propagate_energy(state, -sign_current * sigma_mag,
                                                               atmosphere, mu, mars_radius, e_current,
                                                               constant_bank=True, return_final=True)

            # Compute predicted cross-range (absolute) at terminal for both scenarios
            _, _, _, RC_pos, _, _, _ = spherical_metrics(initial_phi, initial_theta, phi_pos, theta_pos,
                                                        target.phi, target.theta, mars_radius)
            _, _, _, RC_neg, _, _, _ = spherical_metrics(initial_phi, initial_theta, phi_neg, theta_neg,
                                                        target.phi, target.theta, mars_radius)

            chi_pos = abs(RC_pos)
            chi_neg = abs(RC_neg)
            print(f"Predictor: chi_current={chi_current:.1f} m, chi_pos={chi_pos:.1f} m, chi_neg={chi_neg:.1f} m")

            # If opposite prediction is approximately zero, treat ratio as infinite and reverse now
            if chi_neg < 1e-3:
                ratio = np.inf
            else:
                ratio = abs(chi_current) / chi_neg

            # Remaining reversals
            n = max(1, self.n_reversals_remain)
            # Gain K
            K = (abs(chi_current) / max(chi_f, 1e-9)) ** (1.0 / n)
            K = max(self.k_min, min(self.k_max, K))

            # Decision: reverse if current/opposite > K*(1+hyst) and dwell time passed
            if ratio > K * (1.0 + self.hysteresis):
                # Reverse (flip sign)
                desired_sign = -sign_current
                # decrement remaining reversals
                self.n_reversals_remain = max(0, self.n_reversals_remain - 1)
                self.last_reverse_time = t
            else:
                desired_sign = sign_current

        # Compose desired bank (signed)
        desired_sigma = desired_sign * sigma_mag

        # Simplified: directly command desired sign (no rate limiting)
        sigma_cmd = np.clip(desired_sigma, -np.pi, np.pi)

        # Update internal state
        self.sigma_prev = sigma_cmd
        self.last_update_time = t

        return sigma_cmd
    
    def _propagate_energy(self, state, sigma0, atmosphere, mu, mars_radius, e0,
                          constant_bank: bool = False, return_final: bool = False):
        """
        Propagate trajectory using energy as independent variable
        
        Args:
            state: Current state
            sigma0: Initial bank angle for this propagation [rad]
            atmosphere: Atmosphere model object
            mu: Gravitational parameter [m^3/s^2]
            mars_radius: Planet radius [m]
            e0: Initial energy [J/kg]
        
        Returns:
            s_pred: Predicted surface distance traveled [m]
        """
        # Vehicle parameters (from VehicleProperties in main.py)
        beta = 379.0  # Ballistic coefficient [kg/m^2]
        L_over_D = 0.54  # Lift-to-drag ratio
        omega = 7.088e-5  # Mars rotation rate [rad/s]
        
        s_pred = 0.0

        # Unpack state
        r = state.r
        theta = state.theta
        phi = state.phi
        V = state.V
        gamma = state.gamma
        psi = state.psi
        
        e = e0
        #print(f"  Propagating from e0={e0:.1f} J/kg with sigma0={np.rad2deg(sigma0):.2f}°")
        
        # Propagate until terminal energy
        while e < self.e_f:
            # Adaptive energy step: reduce near terminal energy for accuracy
            e_remaining = self.e_f - e
            if e_remaining < 100000:  # Start reducing in last 100 kJ/kg
                # Smoothly transition from self.de down to 100 J/kg
                # This gives finer resolution as we approach terminal energy
                de_step = max(100.0, min(self.de, e_remaining / 20))
            else:
                de_step = self.de
            
            # Bank profile: linear unless constant_bank is requested
            if constant_bank:
                sigma = sigma0
            else:
                den = max(1e-9, (self.e_f - e0))
                sigma = sigma0 + (e - e0) / den * (self.sigma_f - sigma0)
            #print(sigma)
            
            # Atmosphere
            h = r - mars_radius
            rho = atmosphere.density(h)
            #print(f"h={h:.1f} m, rho={rho:.6f} kg/m³")
            #rho = 0.020*np.exp(-h / 11100) #self.mars.rho0 * np.exp(-h / self.mars.H_s)
            
            # Aerodynamic forces
            D = rho * V**2 / (2 * beta)
            L = L_over_D * D
            g = mu / r**2
            #print(f"D={D:.2f} m/s², L={L:.2f} m/s², g={g:.2f} m/s²")
            
            # 3DOF equations of motion
            # Centrifugal acceleration terms
            V_h = V * np.cos(gamma)  # Horizontal velocity
        
            # State derivatives
            r_dot = V * np.sin(gamma)
        
            theta_dot = (V_h * np.sin(psi)) / (r * np.cos(phi))
        
            phi_dot = (V_h * np.cos(psi)) / r
        
            V_dot = -D - g * np.sin(gamma) + omega**2 * r * np.cos(phi) * \
                (np.sin(gamma) * np.cos(phi) - np.cos(gamma) * np.sin(phi) * np.cos(psi))
        
            gamma_dot = (L * np.cos(sigma) / V) + (V / r - g / V) * np.cos(gamma) + \
                    2 * omega * np.cos(phi) * np.sin(psi) + \
                    (omega**2 * r / V) * np.cos(phi) * \
                    (np.cos(gamma) * np.cos(phi) + np.sin(gamma) * np.sin(phi) * np.cos(psi))
        
            psi_dot = (L * np.sin(sigma)) / (V * np.cos(gamma)) + \
                  (V_h / r) * np.sin(psi) * np.tan(phi) + \
                  2 * omega * (np.sin(phi)-np.tan(gamma) * np.cos(phi) * np.cos(psi)) + \
                  (omega**2 * r / (V * np.cos(gamma))) * np.sin(phi) * np.cos(phi) * np.sin(psi)
            
            # Energy rate
            e_dot = D * V
            #print(f"    e={e:.1f} J/kg, e_dot={e_dot:.1f} J/kg/s")
            
            if abs(e_dot) < 1e-9:
                # Energy not changing significantly, break to avoid division issues
                break
            
            # Time step based on energy step, capped for stability
            #dt = min(de_step / e_dot, 0.5)
            dt= de_step / e_dot
            #print(f"dt={dt:.5f}s, e_dot={e_dot:.2f} J/kg/s")
            
            # Euler integration
            r = r + r_dot * dt
            V = V + V_dot * dt
            gamma = gamma + gamma_dot * dt
            psi = psi + psi_dot * dt
            theta = theta + theta_dot * dt
            phi = phi + phi_dot * dt
            
            # Accumulate surface distance
            s_pred = s_pred + V * np.cos(gamma) * dt
            
            # Update energy with adaptive step
            e = e + de_step
            
            # Safety check: prevent infinite loop
            if r < mars_radius:
                break
        
        if return_final:
            return s_pred, theta, phi
        return s_pred
    
    def reset(self):
        """Reset guidance state"""
        super().reset()
        self.sigma0_guess = np.deg2rad(100.0)
        self.sigma_prev = 0.0
    
    def __repr__(self):
        return (f"LNPCGuidance(sigma_f={np.rad2deg(self.sigma_f):.1f}°, "
                f"e_f={self.e_f:.1e} J/kg, activation_t={self.activation_time:.0f}s)")
