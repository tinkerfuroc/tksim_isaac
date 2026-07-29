from __future__ import annotations

from typing import Mapping, Protocol

from .command_mux import JointCommand


class SimulationBackend(Protocol):
    """Contract implemented independently by the Gazebo and Isaac gateways."""

    @property
    def simulation_time(self) -> float:
        """Current simulation time [s]."""

    def step(self) -> None:
        """Advance one CPU-PhysX step while the Isaac timeline is playing."""

    def command_joints(self, command: JointCommand) -> None:
        """Apply the single hardware-gateway JointState command."""

    def parity_state(self) -> Mapping[str, object]:
        """Return only observations available to physical robot software."""

    def truth_state(self, evaluator_token: object) -> Mapping[str, object]:
        """Return hidden ground truth to an authorized evaluator."""
