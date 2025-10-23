"""
Constant bank angle guidance (simplest example)
"""

import numpy as np
from .base_guidance import BaseGuidance


class ConstantBankGuidance(BaseGuidance):
    """
    Simple constant bank angle guidance
    
    This is the simplest possible guidance law - just maintains a constant
    bank angle throughout the trajectory. Good for testing and as a baseline.
    """
    
    def __init__(self, bank_angle_deg: float = 0.0):
        """
        Initialize constant bank guidance
        
        Args:
            bank_angle_deg: Constant bank angle to maintain [degrees]
        """
        super().__init__(name=f"ConstantBank_{bank_angle_deg}deg")
        self.bank_angle = np.deg2rad(bank_angle_deg)
    
    def compute_bank_angle(self, t: float, state, target, mars_radius: float, **kwargs) -> float:
        """
        Return the constant bank angle (ignores state)
        
        Args:
            t: Current time [s]
            state: Current vehicle state (ignored for constant bank)
            target: Target conditions (ignored for constant bank)
            mars_radius: Mars radius [m] (ignored for constant bank)
            **kwargs: Additional parameters (ignored)
        
        Returns:
            sigma: Commanded bank angle [rad]
        """
        return self.bank_angle
    
    def __repr__(self):
        return f"ConstantBankGuidance(bank_angle={np.rad2deg(self.bank_angle):.1f} deg)"
