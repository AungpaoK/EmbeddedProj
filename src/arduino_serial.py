"""Arduino Motion serial startup helpers.

These helpers deliberately accept a pySerial-like object so the reset and
encoder handshake can be exercised without ROS or physical hardware.
"""

from dataclasses import dataclass
import time
from typing import Callable


@dataclass(frozen=True)
class SerialStartupAttempt:
    """Outcome of one DTR reset followed by an encoder-frame wait."""

    frame: tuple[int, int] | None
    received_data: bool
    samples: tuple[str, ...]
    reset_error: str | None = None


def parse_encoder_line(raw: bytes | str) -> tuple[int, int] | None:
    """Return encoder ticks for a complete ENCODER line, otherwise None."""
    if isinstance(raw, bytes):
        line = raw.decode("utf-8", errors="replace").strip()
    else:
        line = str(raw).strip()

    if not line.startswith("ENCODER:"):
        return None

    values = line[len("ENCODER:"):].split(",")
    if len(values) != 2:
        return None
    try:
        return int(values[0]), int(values[1])
    except ValueError:
        return None


def wait_for_encoder(
    ser,
    timeout_seconds: float,
    *,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    idle_sleep_seconds: float = 0.01,
) -> SerialStartupAttempt:
    """Read until one valid encoder frame arrives or the deadline expires.

    Read only bytes already waiting in the driver. ``readline()`` can outlive
    its timeout when a device continuously sends data without a newline, which
    would otherwise defeat the bounded startup wait.
    """
    deadline = monotonic() + max(0.0, timeout_seconds)
    received_data = False
    samples: list[str] = []
    pending = bytearray()
    max_pending_bytes = 1024

    while monotonic() < deadline:
        available = ser.in_waiting
        if available <= 0:
            remaining = deadline - monotonic()
            if remaining > 0:
                sleep(min(idle_sleep_seconds, remaining))
            continue

        raw = ser.read(min(available, max_pending_bytes))
        if not raw:
            continue
        received_data = True
        pending.extend(raw.encode("utf-8") if isinstance(raw, str) else raw)

        while b"\n" in pending:
            raw_line, _, remaining_bytes = pending.partition(b"\n")
            pending = bytearray(remaining_bytes)
            line = raw_line.decode("utf-8", errors="replace").strip()
            frame = parse_encoder_line(line)
            if frame is not None:
                return SerialStartupAttempt(frame, received_data, tuple(samples))
            if line and len(samples) < 3:
                samples.append(line[:160])

        if len(pending) > max_pending_bytes:
            if len(samples) < 3:
                sample = pending[:160].decode("utf-8", errors="replace").strip()
                samples.append(f"unterminated serial data: {sample!r}")
            pending.clear()

    if pending and len(samples) < 3:
        sample = pending[:160].decode("utf-8", errors="replace").strip()
        samples.append(f"unterminated serial data: {sample!r}")

    return SerialStartupAttempt(None, received_data, tuple(samples))


def reset_and_wait_for_encoder(
    ser,
    *,
    timeout_seconds: float = 10.0,
    attempts: int = 2,
    dtr_pulse_seconds: float = 0.25,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[SerialStartupAttempt, ...]:
    """Pulse DTR and wait for a valid encoder frame, retrying once by default."""
    if attempts < 1:
        raise ValueError("attempts must be at least one")

    outcomes: list[SerialStartupAttempt] = []
    for _ in range(attempts):
        reset_errors: list[str] = []
        try:
            ser.reset_input_buffer()
        except Exception as exc:
            reset_errors.append(f"could not clear serial input buffer: {exc}")

        try:
            ser.dtr = False
            sleep(dtr_pulse_seconds)
            ser.dtr = True
        except Exception as exc:
            reset_errors.append(f"could not pulse DTR: {exc}")
            try:
                ser.dtr = True
            except Exception:
                pass

        outcome = wait_for_encoder(
            ser,
            timeout_seconds,
            monotonic=monotonic,
            sleep=sleep,
        )
        if reset_errors:
            outcome = SerialStartupAttempt(
                outcome.frame,
                outcome.received_data,
                outcome.samples,
                "; ".join(reset_errors),
            )
        outcomes.append(outcome)
        if outcome.frame is not None:
            break

    return tuple(outcomes)
