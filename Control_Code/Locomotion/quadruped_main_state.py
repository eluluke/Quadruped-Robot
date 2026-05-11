"""
quadruped_main_state.py

Shared stop flag so modules can see Ctrl+C state without circular imports.
"""

STOP_REQUESTED_REF = {"stop": False}
