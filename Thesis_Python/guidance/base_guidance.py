"""
Base class for guidance strategies
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseGuidance(ABC):
    """
    Abstract base class for entry guidance strategies
    
    All guidance laws must implement the compute_bank_angle method
    which takes the current state and returns the commanded bank angle.
    """
    
    def __init__(self, name: str = "BaseGuidance"):
        """
        Initialize guidance strategy
        
        Args:
            name: Name identifier for this guidance law
        """
        self.name = name
        self.iteration = 0
    
    @abstractmethod
    def compute_bank_angle(self, t: float, state, target, mars_radius: float, **kwargs) -> float:
        """
        Compute the commanded bank angle for the current state
        
        Args:
            t: Current time [s]
            state: Current vehicle state (State object with r, theta, phi, V, gamma, psi)
            target: Target conditions (TargetConditions object)
            mars_radius: Mars radius [m]
            **kwargs: Additional parameters that specific guidance laws may need
                     (e.g., atmospheric data, constraints, etc.)
        
        Returns:
            sigma: Commanded bank angle [rad]
        """
        pass
    
    def reset(self):
        """Reset guidance state (for multiple runs)"""
        self.iteration = 0
    
    def update(self, t: float, state, target, mars_radius: float, **kwargs) -> float:
        """
        Wrapper that calls compute_bank_angle and tracks iterations
        
        Args:
            Same as compute_bank_angle
            
        Returns:
            sigma: Commanded bank angle [rad]
        """
        self.iteration += 1
        return self.compute_bank_angle(t, state, target, mars_radius, **kwargs)
    
    def __repr__(self):
        return f"{self.__class__.__name__}(name='{self.name}')"
