"""
LNPCG (Linear Numerical Predictor-Corrector Guidance) for Mars entry

Based on the energy-domain predictor-corrector method that:
1. Propagates trajectory forward using energy as independent variable
2. Uses Newton-Raphson to find initial bank angle that hits target range
3. Commands linear bank profile between current and final energy
"""

import numpy as np
from .base_guidance import BaseGuidance


class LNPCGuidance(BaseGuidance):
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
                 e_f: float = -2.0e6,
                 activation_time: float = 170.0,
                 max_iter: int = 50,
                 epsilon: float = 100.0,
                 de: float = 100.0,
                 dsigma_deg: float = 10.0):
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
        self.max_iter = max_iter
        self.epsilon = epsilon
        self.de = de
        self.dsigma = np.deg2rad(dsigma_deg)
        
        # Initial guess for bank angle (will be updated during flight)
        self.sigma0_guess = 100.0
        self.sigma_prev = 0.0
        
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
            return 0.0
        
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
            state.phi, state.theta, state.phi, state.theta, 
            target.phi, target.theta, mars_radius
        )
        s_target = Rgo  # Total range-to-go [m]
        
        # Check if we're close to terminal energy
        if abs(e_current - self.e_f) <= 1.0:
            return self.sigma_f
        
        # Bracketing check: does solution exist between 0° and 200°?
        z_0 = self._propagate_energy(state, np.deg2rad(0), atmosphere, 
                                      mu, mars_radius, e_current) - s_target
        z_200 = self._propagate_energy(state, np.deg2rad(200), atmosphere, 
                                        mu, mars_radius, e_current) - s_target
        
        if z_0 * z_200 > 0:
            # Root not bracketed - use previous bank angle
            print(f"  Warning: LNPCG root not bracketed at t={t:.1f}s. Using previous sigma.")
            return self.sigma_prev
        
        # Newton-Raphson iteration to find optimal sigma0
        sigma0_current = self.sigma0_guess
        
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
                break
            
            # Newton-Raphson update
            sigma0_current = sigma0_current - (s_pred - s_target) / dz_dsigma
        
        # Update guess for next iteration
        self.sigma0_guess = sigma0_current
        
        # Compute linear bank profile: sigma(e) = sigma0 + (e-e0)/(ef-e0)*(sigmaf-sigma0)
        sigma_cmd = sigma0_current + (e_current - e_current) / (self.e_f - e_current) * \
                    (self.sigma_f - sigma0_current)
        
        # Simplifies to sigma_cmd = sigma0_current at current time
        # But as energy evolves, this will transition to sigma_f
        sigma_cmd = sigma0_current
        
        # Clip to [-π, π]
        sigma_cmd = np.clip(sigma_cmd, -np.pi, np.pi)
        
        # Store for fallback
        self.sigma_prev = sigma_cmd
        
        return sigma_cmd
    
    def _propagate_energy(self, state, sigma0, atmosphere, mu, mars_radius, e0):
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
        e = e0
        
        # Unpack state
        r = state.r
        theta = state.theta
        phi = state.phi
        V = state.V
        gamma = state.gamma
        psi = state.psi
        
        # Propagate until terminal energy
        while e < self.e_f:
            # Linear bank profile
            sigma = sigma0 + (e - e0) / (self.e_f - e0) * (self.sigma_f - sigma0)
            
            # Atmosphere
            h = r - mars_radius
            rho = atmosphere.density(h)
            
            # Aerodynamic forces
            D = rho * V**2 / (2 * beta)
            L = L_over_D * D
            g = mu / r**2
            
            # 3DOF equations of motion
            rdot = V * np.sin(gamma)
            thetadot = V * np.cos(gamma) * np.sin(psi) / (r * np.cos(phi))
            phidot = V * np.cos(gamma) * np.cos(psi) / r
            
            Vdot = -D - g * np.sin(gamma) + \
                   omega**2 * r * np.cos(phi) * \
                   (np.sin(gamma) * np.cos(phi) - np.cos(gamma) * np.sin(phi) * np.sin(psi))
            
            gammadot = (L * np.cos(sigma)) / V + \
                       (V / r - g / V) * np.cos(gamma) + \
                       2 * omega * np.cos(phi) * np.sin(psi) + \
                       (omega**2 * r / V) * np.cos(phi) * \
                       (np.cos(gamma) * np.cos(phi) + np.sin(gamma) * np.sin(phi) * np.cos(psi))
            
            psidot = (L * np.sin(sigma)) / (V * np.cos(gamma)) + \
                     (V / r) * np.cos(gamma) * np.sin(psi) * np.tan(phi) - \
                     2 * omega * (np.tan(gamma) * np.cos(phi) * np.cos(psi) - np.sin(phi)) + \
                     (omega**2 * r / (V * np.cos(gamma))) * np.sin(phi) * np.cos(phi) * np.sin(psi)
            
            # Energy rate
            e_dot = D * V
            
            if abs(e_dot) < 1e-10:
                # Energy not changing, break to avoid division by zero
                break
            
            # Time step based on energy step
            dt = self.de / e_dot
            
            # Euler integration
            r = r + rdot * dt
            V = V + Vdot * dt
            gamma = gamma + gammadot * dt
            psi = psi + psidot * dt
            theta = theta + thetadot * dt
            phi = phi + phidot * dt
            
            # Accumulate surface distance
            s_pred = s_pred + V * np.cos(gamma) * dt
            
            # Update energy
            e = e + self.de
            
            # Safety check: prevent infinite loop
            if e > 0 or r < mars_radius:
                break
        
        return s_pred
    
    def reset(self):
        """Reset guidance state"""
        super().reset()
        self.sigma0_guess = 0.0
        self.sigma_prev = 0.0
    
    def __repr__(self):
        return (f"LNPCGuidance(sigma_f={np.rad2deg(self.sigma_f):.1f}°, "
                f"e_f={self.e_f:.1e} J/kg, activation_t={self.activation_time:.0f}s)")
