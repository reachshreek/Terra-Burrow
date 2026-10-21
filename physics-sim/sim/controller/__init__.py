"""The bunny's controller machinery. Your control laws go in ../bunny_controller.py.

  interface.py   Inputs, Outputs, commands, HopPlan, HopRecord
  estimator.py   roll/pitch/yaw and stillness from the IMU
  legs.py        per-leg phase logic (rear latch legs, front servo legs + leveling)
  baseline.py    BaselineController: sequencing, telemetry, and the hooks you override
  bound_model.py the steering model: fit "bound hop command -> distance, body turn" from a bound map, and invert it
"""

from .baseline import BaselineController
from .interface import (CONTROL_RATE_HZ, DT, SIDES, BoundCommand, HopCommand, HopPlan, HopRecord,
                        Inputs, Outputs, ServoCommand)

__all__ = ["BaselineController", "CONTROL_RATE_HZ", "DT", "SIDES", "BoundCommand", "HopCommand",
           "HopPlan", "HopRecord", "Inputs", "Outputs", "ServoCommand"]
