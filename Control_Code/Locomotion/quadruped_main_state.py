"""
quadruped_main_state.py

Shared stop flag used by the high-level state machine and controllers.
Keeping it in a tiny module avoids circular imports around Ctrl+C handling.
"""

STOP_REQUESTED_REF = {"stop": False}
