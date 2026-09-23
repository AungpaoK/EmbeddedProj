#!/usr/bin/env python3
"""Small stdlib-only .env loader used by the robot entry points."""

from __future__ import annotations

import os
from pathlib import Path


def _parse_value(raw_value: str) -> str:
    """Parse the common dotenv value forms used by this project."""
    value = raw_value.strip()

    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]

    # Support unquoted inline comments while keeping values such as URLs intact.
    for index, char in enumerate(value):
        if char == "#" and (index == 0 or value[index - 1].isspace()):
            return value[:index].rstrip()
    return value


def load_dotenv(path: str | os.PathLike[str] | None = None) -> Path | None:
    """Load a project ``.env`` file without overriding explicit environment vars.

    ``ENV_FILE`` can point to another dotenv file. Values already exported by the
    shell always win, which keeps command-line overrides working as expected.
    """
    if path is None:
        configured_path = os.environ.get("ENV_FILE")
        path = configured_path or Path(__file__).resolve().parent.parent / ".env"

    env_path = Path(path).expanduser()
    if not env_path.is_file():
        return None

    for line_number, line in enumerate(env_path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            continue

        key, raw_value = line.split("=", 1)
        key = key.strip()
        if not key or any(char not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_" for char in key):
            raise ValueError(f"Invalid .env key on line {line_number}: {key!r}")
        os.environ.setdefault(key, _parse_value(raw_value))

    return env_path

