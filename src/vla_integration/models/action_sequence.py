"""
ActionSequence Model
Represents an ordered list of ROS 2 actions generated from a natural language command, with dependencies and execution context
"""

from dataclasses import dataclass
from typing import List, Dict, Any
from datetime import datetime
from .action_step import ActionStep


@dataclass
class ActionSequence:
    """
    Represents an ordered sequence of actions with dependencies.
    """
    id: str
    command_id: str
    actions: List[ActionStep]
    status: str  # pending, executing, completed, failed
    dependencies: List[str]  # action IDs that must complete before this one
    created_at: datetime

    def __post_init__(self):
        """Validate the ActionSequence after initialization."""
        if len(self.actions) == 0:
            raise ValueError("ActionSequence must contain at least one action")

        # Check that all dependency references exist within the sequence
        action_ids = {action.id for action in self.actions}
        for dep_id in self.dependencies:
            if dep_id not in action_ids:
                raise ValueError(f"Dependency reference {dep_id} does not exist in the sequence")

        # Validate status is one of the allowed values
        allowed_statuses = {"pending", "executing", "completed", "failed"}
        if self.status not in allowed_statuses:
            raise ValueError(f"Status must be one of {allowed_statuses}")