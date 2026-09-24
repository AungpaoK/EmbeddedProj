"""Turn signal state filtering for the Raspberry Pi motion bridge."""

from dataclasses import dataclass


TURN_OFF = "OFF"
TURN_LEFT = "LEFT"
TURN_RIGHT = "RIGHT"


@dataclass
class TurnIndicatorController:
    """Debounce turn intent so small steering corrections do not flash LEDs."""

    turn_on_threshold: float = 0.30
    turn_off_threshold: float = 0.18
    settle_seconds: float = 0.30
    state: str = TURN_OFF
    _candidate: str | None = None
    _candidate_since: float = 0.0

    def force(self, signal: str) -> None:
        """Synchronize state after an explicit, non-streamed motion command."""
        self.state = signal
        self._candidate = None

    def update(self, angular_z: float, now: float) -> str | None:
        """Return a new signal only after the requested state settles."""
        magnitude = abs(angular_z)
        if self.state == TURN_OFF:
            if magnitude < self.turn_on_threshold:
                desired = TURN_OFF
            else:
                # The rear matrix is mounted facing backward, so its visual
                # left/right must be swapped to match the robot's travel.
                desired = TURN_RIGHT if angular_z > 0 else TURN_LEFT
        elif magnitude < self.turn_off_threshold:
            desired = TURN_OFF
        elif magnitude < self.turn_on_threshold:
            desired = self.state
        else:
            desired = TURN_RIGHT if angular_z > 0 else TURN_LEFT

        if desired == self.state:
            self._candidate = None
            return None

        if desired != self._candidate:
            self._candidate = desired
            self._candidate_since = now
            return None

        if now - self._candidate_since < self.settle_seconds:
            return None

        self.state = desired
        self._candidate = None
        return self.state
