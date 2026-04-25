"""Wrapper for invoking the hosted acs_replay binary."""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Sequence

from .paths import find_replay_binary


def replay_command(
    input_csv: Path | None,
    passthrough_args: Sequence[str] = (),
    binary: Path | None = None,
) -> list[str]:
    command = [str(find_replay_binary(binary))]
    if input_csv is not None:
        command.append(str(input_csv))
    command.extend(passthrough_args)
    return command


def run_replay(
    input_csv: Path | None,
    passthrough_args: Sequence[str] = (),
    binary: Path | None = None,
) -> int:
    return subprocess.call(replay_command(input_csv, passthrough_args, binary))

