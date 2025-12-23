"""
ActionStep Model
Represents an individual action step within an action sequence
"""

from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class ActionStep:
    """
    Represents a single action step in an action sequence.
    """
    id: str
    action_type: str  # navigate, detect_object, pick_place, etc.
    parameters: Dict[str, Any]
    timeout: int  # max time to execute in seconds
    retry_count: int  # number of retry attempts
    status: str  # pending, executing, completed, failed

    def __post_init__(self):
        """Validate the ActionStep after initialization."""
        if self.timeout <= 0:
            raise ValueError("timeout must be positive")

        if self.retry_count < 0:
            raise ValueError("retry_count must be non-negative")

        # Validate status is one of the allowed values
        allowed_statuses = {"pending", "executing", "completed", "failed"}
        if self.status not in allowed_statuses:
            raise ValueError(f"Status must be one of {allowed_statuses}")