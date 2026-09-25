"""Map an intentional turn direction to the rear matrix's displayed side."""

TURN_OFF = "OFF"
TURN_LEFT = "LEFT"
TURN_RIGHT = "RIGHT"


def display_signal_for_intent(direction: str, *, invert_steer: bool = False) -> str:
    """Return the indicator side for an explicit turn intent.

    Positive yaw means a left turn. The rear-facing matrix is mirrored, and
    the motion bridge may invert steering to match the robot's drive motors.
    """
    if direction == TURN_OFF:
        return TURN_OFF
    if direction not in (TURN_LEFT, TURN_RIGHT):
        raise ValueError(f"Unsupported turn intent: {direction!r}")

    yaw_sign = 1 if direction == TURN_LEFT else -1
    if invert_steer:
        yaw_sign *= -1
    return TURN_RIGHT if yaw_sign > 0 else TURN_LEFT
