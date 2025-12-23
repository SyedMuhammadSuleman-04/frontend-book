"""
TaskFeedback Model
Represents feedback messages during task execution
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
from datetime import datetime


@dataclass
class TaskFeedback:
    """
    Represents feedback message during task execution.
    """
    id: str
    task_execution_id: str
    message: str
    level: str  # info, warning, error
    timestamp: datetime
    data: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate the TaskFeedback after initialization."""
        if self.data is None:
            self.data = {}

        # Validate level is one of the allowed values
        allowed_levels = {"info", "warning", "error"}
        if self.level not in allowed_levels:
            raise ValueError(f"Level must be one of {allowed_levels}")