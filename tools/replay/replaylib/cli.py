"""Command dispatch helpers for replay scripts."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Iterable, Sequence

from .paths import EXPERIMENTS_DIR, REPLAY_DIR, ROOT


def bootstrap_paths() -> None:
    """Put replay helpers and archived experiment directories on sys.path."""

    candidates = [ROOT, REPLAY_DIR]
    if EXPERIMENTS_DIR.exists():
        candidates.extend(path for path in EXPERIMENTS_DIR.iterdir() if path.is_dir())
    for path in candidates:
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def archived_script(relative_path: str | Path) -> Path:
    path = EXPERIMENTS_DIR / relative_path
    if not path.exists():
        raise FileNotFoundError(f"Archived replay script not found: {path}")
    return path


def run_script(path: Path, args: Sequence[str] = (), display_name: str | None = None) -> int:
    bootstrap_paths()
    old_argv = sys.argv[:]
    sys.argv = [display_name or str(path), *args]
    try:
        globals_dict = {
            "__name__": "__main__",
            "__file__": str(path),
            "__package__": None,
            "__cached__": None,
        }
        exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"), globals_dict)
    except SystemExit as exc:
        code = exc.code
        if code is None:
            return 0
        if isinstance(code, int):
            return code
        print(code, file=sys.stderr)
        return 1
    finally:
        sys.argv = old_argv
    return 0


def run_archived(
    relative_path: str | Path,
    args: Sequence[str] = (),
    display_name: str | None = None,
) -> int:
    return run_script(archived_script(relative_path), args, display_name)


def format_choices(choices: Iterable[tuple[str, str]]) -> str:
    return "\n".join(f"  {key:<28} {value}" for key, value in sorted(choices))
