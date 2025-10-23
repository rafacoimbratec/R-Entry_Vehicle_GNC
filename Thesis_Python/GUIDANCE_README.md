# Modular Guidance Architecture

## Overview

This setup separates **dynamics** (3DOF physics) from **guidance** (control strategy), making it easy to implement and compare different guidance laws.

## File Structure

```
Thesis_Python/
├── main.py                      # Core 3DOF dynamics & simulation
├── spherical_metrics.py         # Range/distance calculations
├── run_simulation.py            # Example runner (compare guidance laws)
│
└── guidance/                    # Guidance strategies package
    ├── __init__.py
    ├── base_guidance.py         # Abstract base class
    └── constant_bank.py         # Example: constant bank angle
```

## How It Works

### 1. Base Guidance Class (`guidance/base_guidance.py`)

All guidance laws inherit from `BaseGuidance` and must implement:

```python
def compute_bank_angle(self, t, state, target, mars_radius, **kwargs):
    """Returns commanded bank angle [rad]"""
    pass
```

### 2. Implementing a New Guidance Law

Create a new file in `guidance/`, for example `guidance/my_guidance.py`:

```python
from .base_guidance import BaseGuidance
import numpy as np

class MyGuidance(BaseGuidance):
    def __init__(self, some_parameter):
        super().__init__(name="MyGuidance")
        self.param = some_parameter
    
    def compute_bank_angle(self, t, state, target, mars_radius, **kwargs):
        # Your guidance logic here
        # Access state: state.h, state.V, state.gamma, etc.
        # Access target: target.h, target.phi, target.theta, etc.
        
        sigma = # ... your calculation
        return sigma
```

Then add it to `guidance/__init__.py`:
```python
from .my_guidance import MyGuidance
__all__ = [..., 'MyGuidance']
```

### 3. Running Simulations

#### Option A: Use `run_simulation.py` (recommended for comparisons)

```python
from guidance import MyGuidance
from run_simulation import run_with_guidance

# Create guidance object
guidance = MyGuidance(some_parameter=value)

# Run simulation
results, sim = run_with_guidance(guidance)

# Plot results
sim.plot_results(results)
```

#### Option B: Use `main.py` directly (for single runs)

```python
from main import *
from guidance import MyGuidance

# Setup
mars = MarsConstants()
vehicle = VehicleProperties()
ic = InitialConditions()
target = TargetConditions()
constraints = PathConstraints()
sim = ReentrySimulation(mars, vehicle, ic, target, constraints)

# Create guidance
guidance = MyGuidance(some_parameter=value)

# Run with guidance
results = sim.run(guidance=guidance, time_step=0.1, stop_time=7000.0)

# Or run with constant bank (old way still works)
results = sim.run(bank_angle=np.deg2rad(30.0), time_step=0.1)
```

## Example: Compare Multiple Guidance Laws

```python
from guidance import ConstantBankGuidance
from run_simulation import compare_guidance_laws, plot_comparison

# Define guidance laws to test
guidance_laws = [
    ConstantBankGuidance(bank_angle_deg=0.0),
    ConstantBankGuidance(bank_angle_deg=30.0),
    ConstantBankGuidance(bank_angle_deg=60.0),
]

# Run all simulations
results_dict = compare_guidance_laws(guidance_laws)

# Plot comparison
fig = plot_comparison(results_dict)
plt.show()
```

## Next Steps

Add more guidance laws to `guidance/` directory:
- `apollo_skip.py` - Apollo skip entry guidance
- `predictor_corrector.py` - Predictor-corrector
- `reference_tracking.py` - Reference trajectory tracking
- `numerical_guidance.py` - Optimization-based

Each can be tested independently and compared easily!
