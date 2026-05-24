"""
quadruped_utils.py

Small shared helpers that are not specific to hardware, trajectory geometry,
or Xbox input.
"""

from __future__ import annotations

import math
import queue
import sys
import threading
import time
from typing import Optional


TERMINAL_QUEUE: queue.Queue[str] = queue.Queue()


def terminal_input_worker() -> None:
    """Read terminal lines into a shared queue."""
    while True:
        try:
            line = sys.stdin.readline()
            if line == "":
                return
            TERMINAL_QUEUE.put(line.strip())
        except Exception:
            return


def start_terminal_input_thread() -> threading.Thread:
    """Start the shared nonblocking terminal input thread."""
    thread = threading.Thread(target=terminal_input_worker, daemon=True)
    thread.start()
    return thread


def get_terminal_command() -> Optional[str]:
    """Return one queued terminal command, or None."""
    try:
        return TERMINAL_QUEUE.get_nowait()
    except queue.Empty:
        return None


def wait_for_terminal_command(prompt: str) -> str:
    """Block until the terminal input thread receives one command."""
    print(prompt, end="", flush=True)
    while True:
        cmd = get_terminal_command()
        if cmd is not None:
            print(cmd)
            return cmd.strip().lower()
        time.sleep(0.02)


def limit_rate(current: float, target: float, max_delta: float) -> float:
    """Limit one scalar value's step toward a target."""
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def smoothstep(u: float) -> float:
    """Cosine smoothstep over u in [0, 1]."""
    u = max(0.0, min(1.0, u))
    return 0.5 * (1.0 - math.cos(math.pi * u))
