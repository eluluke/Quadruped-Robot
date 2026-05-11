"""
quadruped_utils.py

Small shared helpers.
"""

import math
import time
import queue
import sys
import threading

TERMINAL_QUEUE = queue.Queue()


def terminal_input_worker():
    while True:
        try:
            line = sys.stdin.readline()
            if line == "":
                return
            TERMINAL_QUEUE.put(line.strip())
        except Exception:
            return


def start_terminal_input_thread():
    thread = threading.Thread(target=terminal_input_worker, daemon=True)
    thread.start()
    return thread


def get_terminal_command():
    try:
        return TERMINAL_QUEUE.get_nowait()
    except queue.Empty:
        return None


def wait_for_terminal_command(prompt: str) -> str:
    print(prompt, end="", flush=True)
    while True:
        cmd = get_terminal_command()
        if cmd is not None:
            print(cmd)
            return cmd.strip().lower()
        time.sleep(0.02)


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadband) / (1.0 - deadband)


def limit_rate(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def smoothstep(u: float) -> float:
    return 0.5 * (1.0 - math.cos(math.pi * u))


def table_index_from_phase(phase: float, table_len: int) -> int:
    return int((phase % 1.0) * table_len) % table_len
